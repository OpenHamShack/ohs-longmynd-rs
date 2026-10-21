//! ts.c's service-name decoding: the DVB character-set selector (EN 300 468
//! annex A) and iconv to UTF-8, reproduced without iconv (Windows has none).
//!
//! The tables in charset_tables.rs are glibc's own conversions, enumerated
//! byte by byte by tools/gen-charsets.sh, so the output is the C's output.
//! The C uses whatever iconv produces only if it converted the whole name;
//! otherwise it keeps the raw bytes (after the selector), and so does this.
//!
//! Not reproduced: EUC-KR, EUC-CN (GB2312) and Big5, selectors 0x12-0x14.
//! Their tables are tens of thousands of entries for a case amateur DATV
//! does not have; names in them are passed through raw, as the C does for
//! any name iconv cannot convert.

use crate::charset_tables::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Charset {
    Iso6937,
    Iso8859(u8),
    /// "ISO-10646": glibc reads it as UCS-4, big-endian.
    Ucs4,
    /// "ISO-10646/UTF8/"
    Utf8,
    /// EUC-KR, EUC-CN, BIG5: not converted (see above).
    Unsupported,
}

/// dvbCharCodeA3Lookup: the first byte of a name, 0x01-0x1F.
pub fn a3(b: u8) -> Option<Charset> {
    match b {
        0x01..=0x07 => Some(Charset::Iso8859(b + 4)),
        0x09..=0x0B => Some(Charset::Iso8859(b + 4)),
        0x11 => Some(Charset::Ucs4),
        0x12..=0x14 => Some(Charset::Unsupported),
        0x15 => Some(Charset::Utf8),
        _ => None,
    }
}

/// dvbCharCodeA4Lookup: the third byte after 0x10 0x00.
pub fn a4(b: u8) -> Option<Charset> {
    match b {
        0x01..=0x0B => Some(Charset::Iso8859(b)),
        0x0D..=0x0F => Some(Charset::Iso8859(b)),
        0x11 => Some(Charset::Ucs4),
        0x12..=0x14 => Some(Charset::Unsupported),
        0x15 => Some(Charset::Utf8),
        _ => None,
    }
}

fn push_utf8(out: &mut Vec<u8>, cp: u32) {
    // glibc encodes up to 0x7fffffff, in the original 6-byte UTF-8
    if cp < 0x80 {
        out.push(cp as u8);
    } else if cp < 0x800 {
        out.extend_from_slice(&[0xc0 | (cp >> 6) as u8, 0x80 | (cp & 0x3f) as u8]);
    } else if cp < 0x10000 {
        out.extend_from_slice(&[0xe0 | (cp >> 12) as u8, 0x80 | ((cp >> 6) & 0x3f) as u8, 0x80 | (cp & 0x3f) as u8]);
    } else if cp < 0x20_0000 {
        out.extend_from_slice(&[
            0xf0 | (cp >> 18) as u8,
            0x80 | ((cp >> 12) & 0x3f) as u8,
            0x80 | ((cp >> 6) & 0x3f) as u8,
            0x80 | (cp & 0x3f) as u8,
        ]);
    } else if cp < 0x400_0000 {
        out.extend_from_slice(&[
            0xf8 | (cp >> 24) as u8,
            0x80 | ((cp >> 18) & 0x3f) as u8,
            0x80 | ((cp >> 12) & 0x3f) as u8,
            0x80 | ((cp >> 6) & 0x3f) as u8,
            0x80 | (cp & 0x3f) as u8,
        ]);
    } else {
        out.extend_from_slice(&[
            0xfc | (cp >> 30) as u8,
            0x80 | ((cp >> 24) & 0x3f) as u8,
            0x80 | ((cp >> 18) & 0x3f) as u8,
            0x80 | ((cp >> 12) & 0x3f) as u8,
            0x80 | ((cp >> 6) & 0x3f) as u8,
            0x80 | (cp & 0x3f) as u8,
        ]);
    }
}

fn single_table(n: u8) -> Option<&'static [u32; 256]> {
    Some(match n {
        1 => &ISO8859_1,
        2 => &ISO8859_2,
        3 => &ISO8859_3,
        4 => &ISO8859_4,
        5 => &ISO8859_5,
        6 => &ISO8859_6,
        7 => &ISO8859_7,
        8 => &ISO8859_8,
        9 => &ISO8859_9,
        10 => &ISO8859_10,
        11 => &ISO8859_11,
        13 => &ISO8859_13,
        14 => &ISO8859_14,
        15 => &ISO8859_15,
        _ => return None,
    })
}

/// iconv(UTF-8 <- charset) over the whole input: Some(converted) when all
/// of it converted, None when iconv would have stopped short.
pub fn to_utf8(charset: Charset, input: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(input.len() * 2);
    match charset {
        Charset::Utf8 => return Some(input.to_vec()), // valid or not, the bytes come out unchanged
        Charset::Unsupported => return None,
        Charset::Iso8859(n) => {
            let table = single_table(n)?;
            for &b in input {
                let cp = table[b as usize];
                if cp == INVALID {
                    return None;
                }
                push_utf8(&mut out, cp);
            }
        }
        Charset::Iso6937 => {
            let mut i = 0;
            while i < input.len() {
                let cp = ISO6937_SINGLE[input[i] as usize];
                if cp == INVALID {
                    return None;
                }
                if cp == PREFIX {
                    let base = *input.get(i + 1)?; // a trailing diacritic: incomplete
                    let p = input[i];
                    let found = ISO6937_PAIRS.binary_search_by(|&(a, b, _)| (a, b).cmp(&(p, base))).ok()?;
                    push_utf8(&mut out, ISO6937_PAIRS[found].2);
                    i += 2;
                } else {
                    push_utf8(&mut out, cp);
                    i += 1;
                }
            }
        }
        Charset::Ucs4 => {
            if input.len() % 4 != 0 {
                return None;
            }
            for c in input.chunks(4) {
                let cp = u32::from_be_bytes([c[0], c[1], c[2], c[3]]);
                if cp > 0x7fff_ffff || (0xd800..=0xdfff).contains(&cp) {
                    return None;
                }
                push_utf8(&mut out, cp);
            }
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iso6937_diacritic_and_plain() {
        assert_eq!(to_utf8(Charset::Iso6937, b"GB3HV"), Some(b"GB3HV".to_vec()));
        assert_eq!(to_utf8(Charset::Iso6937, &[0xc2, b'e']), Some("é".as_bytes().to_vec()));
        assert_eq!(to_utf8(Charset::Iso6937, &[0xc2]), None, "a trailing diacritic is incomplete");
        assert_eq!(to_utf8(Charset::Iso6937, &[0xc2, b'1']), None);
        assert_eq!(to_utf8(Charset::Iso6937, &[0xa4]), None);
    }

    #[test]
    fn iso8859() {
        assert_eq!(to_utf8(Charset::Iso8859(1), &[0xe9]), Some("é".as_bytes().to_vec()));
        assert_eq!(to_utf8(Charset::Iso8859(5), &[0xb0]), Some("А".as_bytes().to_vec()));
    }

    #[test]
    fn ucs4_matches_glibc() {
        assert_eq!(to_utf8(Charset::Ucs4, &[0, 0, 0, 0x41]), Some(b"A".to_vec()));
        assert_eq!(to_utf8(Charset::Ucs4, &[0, 0, 0xd8, 0]), None);
        assert_eq!(to_utf8(Charset::Ucs4, &[0x80, 0, 0, 0]), None);
        assert_eq!(to_utf8(Charset::Ucs4, &[0, 0x41]), None);
        assert_eq!(to_utf8(Charset::Ucs4, &[0x7f, 0xff, 0xff, 0xff]).unwrap().len(), 6);
    }

    #[test]
    fn pairs_are_sorted_for_binary_search() {
        assert!(ISO6937_PAIRS.windows(2).all(|w| (w[0].0, w[0].1) < (w[1].0, w[1].1)));
    }
}
