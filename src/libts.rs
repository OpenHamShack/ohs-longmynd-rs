//! libts.c: the transport-stream parser that feeds the service name,
//! provider, elementary-stream PIDs and null-packet percentage into the
//! status output.
//!
//! [`ts_parse`] is the C's walk, including that after each packet it
//! advances one byte (not 188) and re-synchronises on the next 0x47, so a
//! 0x47 inside a packet can be taken as a packet start; that it parses the
//! FTDI's status bytes as TS; and that the SDT callback may shorten the
//! name lengths it is given (by the character-set selector), which then
//! moves where the parser looks for the next service. Where the C reads
//! past its buffer the bytes here are read as 0 instead.
//! [`ts_parse_fixed`] is the same parse without those faults
//! (--ts-parse-fix).

pub const TS_PACKET_SIZE: u32 = 188;
pub const TS_HEADER_SYNC: u8 = 0x47;
pub const TS_PID_NULL: u32 = 0x1FFF;
pub const TS_TABLE_PAT: u8 = 0x00;
pub const TS_TABLE_PMT: u8 = 0x02;
pub const TS_TABLE_SDT: u8 = 0x42;

const fn crc32_mpeg2_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = (i as u32) << 24;
        let mut k = 0;
        while k < 8 {
            c = if c & 0x8000_0000 != 0 { (c << 1) ^ 0x04c1_1db7 } else { c << 1 };
            k += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
}

static CRC32_MPEG2_TABLE: [u32; 256] = crc32_mpeg2_table();

fn crc32_mpeg2(data: impl Iterator<Item = u8>) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for b in data {
        crc = (crc << 8) ^ CRC32_MPEG2_TABLE[(((crc >> 24) ^ b as u32) & 0xFF) as usize];
    }
    crc
}

/// What ts_parse() reports, as the C's three callbacks.
pub trait TsCallbacks {
    /// An SDT service. Names are given as offsets into `buf` and lengths
    /// the callback may change (the C passes pointers to its own length
    /// variables, and the parser goes on using what the callback left).
    fn sdt_service(&mut self, buf: &[u8], provider_off: usize, provider_len: &mut u32, name_off: usize, name_len: &mut u32);
    fn pmt_pids(&mut self, index: u32, es_pid: u32, es_type: u32);
    fn ts_stats(&mut self, total_count: u32, null_percentage: u32);
}

/// ts_parse(): `buf` is the whole parse buffer; `length` how much of it is
/// this chunk.
pub fn ts_parse(buf: &[u8], length: u32, cb: &mut dyn TsCallbacks) {
    let get = |i: usize| -> u8 { buf.get(i).copied().unwrap_or(0) };
    let mut total: u32 = 0;
    let mut null: u32 = 0;
    let mut ptr: Option<usize> = Some(0);

    while let Some(mut p) = ptr {
        if get(p) != TS_HEADER_SYNC {
            let remaining = length.wrapping_sub(p as u32);
            if remaining <= TS_PACKET_SIZE {
                ptr = None;
                continue;
            }
            let n = (remaining - TS_PACKET_SIZE) as usize;
            let start = p.min(buf.len());
            let end = p.saturating_add(n).min(buf.len());
            match buf[start..end].iter().position(|&b| b == TS_HEADER_SYNC) {
                Some(i) => p = start + i,
                None => {
                    ptr = None;
                    continue;
                }
            }
        }
        packet(buf, p, &mut total, &mut null, cb, false);
        // one byte on, not 188: the next 0x47 is taken as the next packet
        ptr = Some(p + 1);
    }

    let null_percentage = if total > 0 { (100u32.wrapping_mul(null)) / total } else { 0 };
    cb.ts_stats(total, null_percentage);
}

/// The FTDI puts its two status bytes at the head of every 512-byte USB
/// packet. The TS thread strips the first pair of a read but hands the
/// parser the rest as read, so a pair sits at 510, 1022, ... (510 + 512k)
/// of the parse buffer - inside any TS packet that spans a USB packet.
/// --ts-parse-fix takes them out before parsing (the C parses them as TS).
pub fn deframe(buf: &[u8], length: u32) -> Vec<u8> {
    let data = &buf[..(length as usize).min(buf.len())];
    let mut out = Vec::with_capacity(data.len());
    for piece in data.chunks(512) {
        out.extend_from_slice(&piece[..piece.len().min(510)]);
    }
    out
}

/// --ts-parse-fix: the C's parse with its three faults removed (items 13
/// and 14 of docs/improvements.md):
///
/// - the FTDI status bytes are taken out first ([`deframe`]);
/// - after a packet the walk moves on 188 bytes, so a 0x47 inside a
///   payload is never taken as a packet start; to (re)gain sync it wants a
///   0x47 with another 0x47 188 bytes on (or the end of the chunk there);
/// - it goes on to the end of the chunk, so the last packet is parsed;
/// - SDT services are walked by their own lengths (the descriptor loop
///   length), and the callback's changes to the name lengths do not move
///   where the next service is looked for.
///
/// The callbacks, and what is done with each packet, are the C's.
pub fn ts_parse_fixed(buf: &[u8], length: u32, cb: &mut dyn TsCallbacks) {
    let data = deframe(buf, length);
    let n = data.len();
    let ps = TS_PACKET_SIZE as usize;
    let synced_at = |q: usize| data[q] == TS_HEADER_SYNC && (q + ps >= n || data[q + ps] == TS_HEADER_SYNC);
    let mut total: u32 = 0;
    let mut null: u32 = 0;
    let mut p = 0;
    let mut in_sync = false;
    while p + ps <= n {
        // in sync, a 0x47 where the last packet said the next would be is
        // enough; otherwise (start, or lost) a 0x47 needs another 188 on
        if !(in_sync && data[p] == TS_HEADER_SYNC) && !synced_at(p) {
            match (p + 1..=n - ps).find(|&q| synced_at(q)) {
                Some(q) => p = q,
                None => break,
            }
        }
        in_sync = true;
        packet(&data, p, &mut total, &mut null, cb, true);
        p += ps;
    }
    let null_percentage = if total > 0 { (100u32.wrapping_mul(null)) / total } else { 0 };
    cb.ts_stats(total, null_percentage);
}

/// One packet at `p`: count it, and if it carries a PMT or SDT section
/// whose CRC checks, report what is in it. `fixed` is --ts-parse-fix's SDT
/// walk; everything else is the C's.
fn packet(buf: &[u8], p: usize, total: &mut u32, null: &mut u32, cb: &mut dyn TsCallbacks, fixed: bool) {
    let get = |i: usize| -> u8 { buf.get(i).copied().unwrap_or(0) };
    let pid = ((get(p + 1) as u32 & 0x1F) << 8) | get(p + 2) as u32;
    *total = total.wrapping_add(1);
    if pid == TS_PID_NULL {
        *null = null.wrapping_add(1);
        return;
    }

    let mut content_offset: u32 = 4;
    if (get(p + 3) & 0x20) >> 5 > 0 {
        let af_len = get(p + 4) as u32;
        if af_len == 0 || af_len > 183 {
            return;
        }
        content_offset += af_len;
    }
    let payload_offset = content_offset + 1 + get(p + content_offset as usize) as u32;
    if payload_offset >= TS_PACKET_SIZE - 3 {
        return;
    }
    let pp = p + payload_offset as usize;
    let section_length = ((get(pp + 1) as u32 & 0x0F) << 8) | get(pp + 2) as u32;
    if section_length < 1 || payload_offset + section_length > TS_PACKET_SIZE {
        return;
    }
    let sl = section_length as usize;
    let crc = (get(pp + sl - 1) as u32) << 24 | (get(pp + sl) as u32) << 16 | (get(pp + sl + 1) as u32) << 8 | get(pp + sl + 2) as u32;
    let crc_c = crc32_mpeg2((0..sl - 1).map(|i| get(pp + i)));
    if crc != crc_c {
        return;
    }

    let table = get(pp);
    if table == TS_TABLE_PMT {
        let program_info_length = ((get(pp + 10) as u32 & 0x0F) << 8) | get(pp + 11) as u32;
        let mut offset: u32 = 0;
        let mut index: u32 = 0;
        while 12 + 1 + program_info_length + offset < section_length {
            let es = pp + (12 + program_info_length + offset) as usize;
            let es_type = get(es) as u32;
            let es_pid = ((get(es + 1) as u32 & 0x1F) << 8) | get(es + 2) as u32;
            let es_info_length = ((get(es + 3) as u32 & 0x0F) << 8) | get(es + 4) as u32;
            cb.pmt_pids(index, es_pid, es_type);
            offset += 5 + es_info_length;
            index += 1;
        }
    } else if table == TS_TABLE_PAT {
        // The C walks the programme list only to print it in verbose
        // mode (ts_analyse); longmynd never asks, so nothing to do.
    } else if table == TS_TABLE_SDT && fixed {
        // services from byte 11 of the section to the CRC; each is
        // service_id(2) flags(1) descriptors_loop_length(12 bits of 2),
        // then its descriptors; the service descriptor (0x48) is
        // service_type(1) provider_len(1) provider name_len(1) name
        let end = pp + 3 + sl - 4;
        let mut svc = pp + 11;
        while svc + 5 <= end {
            let loop_len = (((get(svc + 3) & 0x0F) as usize) << 8) | get(svc + 4) as usize;
            let loop_end = (svc + 5 + loop_len).min(end);
            let mut d = svc + 5;
            while d + 2 <= loop_end {
                let (tag, len) = (get(d), get(d + 1) as usize);
                if tag == 0x48 && d + 2 + len <= loop_end && len >= 3 {
                    let provider_len = get(d + 3) as u32;
                    let name_at = d + 4 + provider_len as usize;
                    if name_at < d + 2 + len {
                        // copies: what the callback does to them moves nothing here
                        let (mut pl, mut nl) = (provider_len, get(name_at) as u32);
                        cb.sdt_service(buf, d + 4, &mut pl, name_at + 1, &mut nl);
                    }
                }
                d += 2 + len;
            }
            svc += 5 + loop_len;
        }
    } else if table == TS_TABLE_SDT {
        let mut content_length: u32 = 11;
        let mut table_ptr = pp + 11;
        while content_length < section_length {
            let desc = table_ptr + 5;
            content_length += 5;
            content_length += 3;
            let mut provider_len = get(desc + 3) as u32;
            let mut name_len = get(desc + 3 + 1 + provider_len as usize) as u32;
            cb.sdt_service(buf, desc + 4, &mut provider_len, desc + 4 + 1 + get(desc + 3) as usize, &mut name_len);
            // the lengths as the callback left them: shortened by any
            // character-set selector, which moves the next service
            content_length = content_length.wrapping_add(1);
            content_length = content_length.wrapping_add(provider_len);
            content_length = content_length.wrapping_add(1);
            content_length = content_length.wrapping_add(name_len);
            table_ptr = pp.wrapping_add(content_length as usize);
            content_length = content_length.wrapping_add(11);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_table_matches_the_c_table() {
        // first entries of libts.c's literal table
        assert_eq!(&CRC32_MPEG2_TABLE[..4], &[0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9]);
        assert_eq!(CRC32_MPEG2_TABLE[255], 0xb1f740b4);
        // a CRC over a whole section including its CRC is 0
        let pat = [0x00u8, 0xb0, 0x0d, 0x00, 0x01, 0xc1, 0x00, 0x00, 0x00, 0x01, 0xe1, 0x00];
        let c = crc32_mpeg2(pat.iter().copied());
        let mut with = pat.to_vec();
        with.extend_from_slice(&c.to_be_bytes());
        assert_eq!(crc32_mpeg2(with.iter().copied()), 0);
    }

    struct Rec {
        names: Vec<(Vec<u8>, Vec<u8>)>,
        pids: Vec<(u32, u32, u32)>,
        stats: Vec<(u32, u32)>,
    }
    impl TsCallbacks for Rec {
        fn sdt_service(&mut self, buf: &[u8], po: usize, pl: &mut u32, no: usize, nl: &mut u32) {
            self.names.push((buf[po..po + *pl as usize].to_vec(), buf[no..no + *nl as usize].to_vec()));
        }
        fn pmt_pids(&mut self, i: u32, pid: u32, t: u32) {
            self.pids.push((i, pid, t));
        }
        fn ts_stats(&mut self, total: u32, pct: u32) {
            self.stats.push((total, pct));
        }
    }

    fn packet(pid: u16, section: &[u8]) -> Vec<u8> {
        let mut p = vec![0x47, 0x40 | (pid >> 8) as u8, pid as u8, 0x10, 0x00];
        let crc = crc32_mpeg2(section.iter().copied());
        p.extend_from_slice(section);
        p.extend_from_slice(&crc.to_be_bytes());
        p.resize(188, 0xff);
        p
    }

    fn null_packet() -> Vec<u8> {
        let mut p = vec![0x47, 0x1f, 0xff, 0x10];
        p.resize(188, 0xff);
        p
    }

    #[test]
    fn parses_pmt_sdt_and_null_share() {
        // PMT: program 1, PCR PID 0x100, two streams (H.264 on 0x100, AAC 0x101)
        let pmt = [
            0x02, 0xb0, 0x17, 0x00, 0x01, 0xc1, 0x00, 0x00, 0xe1, 0x00, 0xf0, 0x00, 0x1b, 0xe1, 0x00, 0xf0, 0x00, 0x0f,
            0xe1, 0x01, 0xf0, 0x00,
        ];
        // SDT: one service, provider "BATC", name "GB3HV"
        let mut sdt = vec![0x42, 0xb0, 0x00, 0x00, 0x01, 0xc1, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x01, 0xfc, 0x80, 0x00];
        let desc = [0x48u8, 0x0d, 0x01, 0x04, b'B', b'A', b'T', b'C', 0x05, b'G', b'B', b'3', b'H', b'V'];
        sdt.extend_from_slice(&desc);
        sdt[15] = desc.len() as u8;
        let len = sdt.len() - 3 + 4;
        sdt[2] = len as u8;
        let mut ts = packet(0x1000, &pmt);
        ts.extend(packet(0x11, &sdt));
        ts.extend(null_packet());
        ts.extend(null_packet());
        let mut r = Rec { names: vec![], pids: vec![], stats: vec![] };
        ts_parse(&ts, ts.len() as u32, &mut r);
        assert_eq!(r.pids, vec![(0, 0x100, 0x1b), (1, 0x101, 0x0f)]);
        assert_eq!(r.names, vec![(b"BATC".to_vec(), b"GB3HV".to_vec())]);
        // Not (4, 50): the C's walk never reaches the last packet of a
        // chunk (its re-sync search stops 188 bytes short of the end), and
        // the 'G' (0x47) of "GB3HV" is counted as a packet start. Both are
        // the C's behaviour, kept; see docs/improvements.md.
        assert_eq!(r.stats, vec![(4, 25)]);
    }

    #[test]
    fn fixed_parse_counts_every_packet_and_no_payload_bytes() {
        // the same four packets: the fixed walk sees all four (two nulls,
        // 50 %) and does not take the 'G' of "GB3HV" for a packet
        let pmt = [
            0x02, 0xb0, 0x17, 0x00, 0x01, 0xc1, 0x00, 0x00, 0xe1, 0x00, 0xf0, 0x00, 0x1b, 0xe1, 0x00, 0xf0, 0x00, 0x0f,
            0xe1, 0x01, 0xf0, 0x00,
        ];
        let mut ts = packet(0x1000, &pmt);
        ts.extend(sdt_packet(&[(b"BATC", b"GB3HV")]));
        ts.extend(null_packet());
        ts.extend(null_packet());
        let ts = framed(&ts);
        let mut c = Rec { names: vec![], pids: vec![], stats: vec![] };
        ts_parse(&ts, ts.len() as u32, &mut c);
        assert_ne!(c.stats, vec![(4, 50)], "the C");
        let mut r = Rec { names: vec![], pids: vec![], stats: vec![] };
        ts_parse_fixed(&ts, ts.len() as u32, &mut r);
        assert_eq!(r.stats, vec![(4, 50)]);
        assert_eq!(r.pids, c.pids);
        assert_eq!(r.names, vec![(b"BATC".to_vec(), b"GB3HV".to_vec())]);
    }

    /// TS as the parse thread is handed it: an FTDI status pair at 510,
    /// 1022, ... (the TS thread has stripped only the first pair).
    fn framed(ts: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        for (i, piece) in ts.chunks(510).enumerate() {
            if i > 0 {
                out.extend_from_slice(&[0x32, 0x60]);
            }
            out.extend_from_slice(piece);
        }
        out
    }

    /// An SDT packet with these (provider, name) services, one service
    /// descriptor each.
    fn sdt_packet(services: &[(&[u8], &[u8])]) -> Vec<u8> {
        let mut sdt = vec![0x42, 0xf0, 0x00, 0x00, 0x01, 0xc1, 0x00, 0x00, 0x00, 0x01, 0xff];
        for (i, (provider, name)) in services.iter().enumerate() {
            let mut d = vec![0x48, 0, 0x01, provider.len() as u8];
            d.extend_from_slice(provider);
            d.push(name.len() as u8);
            d.extend_from_slice(name);
            d[1] = (d.len() - 2) as u8;
            sdt.extend_from_slice(&[0x00, 0x01 + i as u8, 0xfc, 0x80, d.len() as u8]);
            sdt.extend_from_slice(&d);
        }
        let len = sdt.len() - 3 + 4;
        sdt[2] = len as u8;
        packet(0x11, &sdt)
    }

    /// What decode_name does to the lengths: drop a one-byte selector.
    struct Shortening(Vec<(Vec<u8>, Vec<u8>)>);
    impl TsCallbacks for Shortening {
        fn sdt_service(&mut self, buf: &[u8], po: usize, pl: &mut u32, no: usize, nl: &mut u32) {
            let take = |o: usize, l: &mut u32| {
                let mut v = buf[o..o + *l as usize].to_vec();
                if v.first().is_some_and(|&b| b < 0x20) {
                    v.remove(0);
                    *l -= 1;
                }
                v
            };
            let p = take(po, pl);
            let n = take(no, nl);
            self.0.push((p, n));
        }
        fn pmt_pids(&mut self, _: u32, _: u32, _: u32) {}
        fn ts_stats(&mut self, _: u32, _: u32) {}
    }

    #[test]
    fn fixed_sdt_walk_is_not_moved_by_a_charset_selector() {
        // two services, the first name with an ISO-8859-9 selector byte
        let ts = framed(&sdt_packet(&[(b"BATC", &[0x05, b'O', b'N', b'E']), (b"BATC", b"TWO")]));
        let mut c = Shortening(vec![]);
        ts_parse(&ts, ts.len() as u32, &mut c);
        assert_eq!(c.0[0], (b"BATC".to_vec(), b"ONE".to_vec()));
        assert_ne!(c.0.get(1), Some(&(b"BATC".to_vec(), b"TWO".to_vec())), "the C finds the second service where it is not");
        let mut r = Shortening(vec![]);
        ts_parse_fixed(&ts, ts.len() as u32, &mut r);
        assert_eq!(r.0, vec![(b"BATC".to_vec(), b"ONE".to_vec()), (b"BATC".to_vec(), b"TWO".to_vec())]);
    }

    #[test]
    fn fixed_parse_skips_the_ftdi_status_bytes() {
        // two nulls, some junk, then a PMT whose section straddles the
        // 510-byte mark of the first USB packet (where the parse buffer
        // keeps the FTDI status pair), then a null
        let pmt = [0x02, 0xb0, 0x12, 0x00, 0x01, 0xc1, 0x00, 0x00, 0xe1, 0x00, 0xf0, 0x00, 0x1b, 0xe1, 0x00, 0xf0, 0x00];
        let mut ts = null_packet();
        ts.extend(null_packet());
        ts.extend(std::iter::repeat(0u8).take(120));
        ts.extend(packet(0x1000, &pmt));
        ts.extend(null_packet());
        let framed = framed(&ts);
        let mut c = Rec { names: vec![], pids: vec![], stats: vec![] };
        ts_parse(&framed, framed.len() as u32, &mut c);
        assert!(c.pids.is_empty(), "the C reads the status bytes as TS: the PMT's CRC fails");
        let mut r = Rec { names: vec![], pids: vec![], stats: vec![] };
        ts_parse_fixed(&framed, framed.len() as u32, &mut r);
        assert_eq!(r.pids, vec![(0, 0x100, 0x1b)]);
        assert_eq!(r.stats, vec![(4, 75)]);
    }
}
