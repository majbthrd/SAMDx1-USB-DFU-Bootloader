/*
    CrossStudio script to patch a SAM Dx1 ELF to work with this bootloader:
    https://github.com/majbthrd/SAMDx1-USB-DFU-Bootloader
    Copyright (C) 2018,2019 Peter Lawrence

    The bootloader expects an application length and CRC32 to be stored
    within the user application (using unused vector table entries).  This 
    tool calculates these quantities and inserts them into the output ELF.

    Permission is hereby granted, free of charge, to any person obtaining a 
    copy of this software and associated documentation files (the "Software"), 
    to deal in the Software without restriction, including without limitation 
    the rights to use, copy, modify, merge, publish, distribute, sublicense, 
    and/or sell copies of the Software, and to permit persons to whom the 
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in 
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
*/

var padding_value = 0xFF;
var app_len_offset = 0x10;
var app_crc_offset = 0x14;

var crc32_table =
[
  0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
  0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
  0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
  0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
  0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
  0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
  0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
  0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
  0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
  0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
  0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
  0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
  0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
  0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
  0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
  0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
  0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
  0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
  0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
  0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
  0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
  0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
  0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
  0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
  0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
  0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
  0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
  0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
  0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
  0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
  0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
  0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
  0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
  0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
  0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
  0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
  0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
  0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
  0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
  0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
  0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
  0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
  0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
  0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
  0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
  0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
  0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
  0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
  0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
  0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
  0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
  0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
  0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
  0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
  0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
  0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
  0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
  0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
  0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
  0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
  0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
  0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
  0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
  0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,
];

var high_lookup =
[
    0,  65, 195, 130, 134, 199,  69,   4,  77,  12, 142, 207, 203, 138,   8,  73,
  154, 219,  89,  24,  28,  93, 223, 158, 215, 150,  20,  85,  81,  16, 146, 211,
  117,  52, 182, 247, 243, 178,  48, 113,  56, 121, 251, 186, 190, 255, 125,  60,
  239, 174,  44, 109, 105,  40, 170, 235, 162, 227,  97,  32,  36, 101, 231, 166,
  234, 171,  41, 104, 108,  45, 175, 238, 167, 230, 100,  37,  33,  96, 226, 163,
  112,  49, 179, 242, 246, 183,  53, 116,  61, 124, 254, 191, 187, 250, 120,  57,
  159, 222,  92,  29,  25,  88, 218, 155, 210, 147,  17,  80,  84,  21, 151, 214,
    5,  68, 198, 135, 131, 194,  64,   1,  72,   9, 139, 202, 206, 143,  13,  76,
  149, 212,  86,  23,  19,  82, 208, 145, 216, 153,  27,  90,  94,  31, 157, 220,
   15,  78, 204, 141, 137, 200,  74,  11,  66,   3, 129, 192, 196, 133,   7,  70,
  224, 161,  35,  98, 102,  39, 165, 228, 173, 236, 110,  47,  43, 106, 232, 169,
  122,  59, 185, 248, 252, 189,  63, 126,  55, 118, 244, 181, 177, 240, 114,  51,
  127,  62, 188, 253, 249, 184,  58, 123,  50, 115, 241, 176, 180, 245, 119,  54,
  229, 164,  38, 103,  99,  34, 160, 225, 168, 233, 107,  42,  46, 111, 237, 172,
   10,  75, 201, 136, 140, 205,  79,  14,  71,   6, 132, 197, 193, 128,   2,  67,
  144, 209,  83,  18,  22,  87, 213, 148, 221, 156,  30,  95,  91,  26, 152, 217,
];

/*
  ElfFile.peekBytes() is PAINFULLY slow when its second arg is large.
  So, it behooves us to incrementally peekBytes at only small chunks.
*/
var chunk_size = 64;

function crc32_calc(crc, address, length)
{
  while (length)
  {
    chunk = (length > chunk_size) ? chunk_size : length;

    peek = ElfFile.peekBytes(address, chunk, false, padding_value);
    length -= chunk;
    address += chunk;
  
    for (i = 0; i < chunk; i++)
    {
      crc = crc32_table[(crc ^ peek[i]) & 0xff] ^ (crc >>> 8);
    }
  }

  return crc >>> 0;
}

function reverse_crc32_calc(crc, address, length)
{
  address += length;

  while (length)
  {
    chunk = (length > chunk_size) ? chunk_size : length;

    address -= chunk;
    peek = ElfFile.peekBytes(address, chunk, false, padding_value);
    length -= chunk;

    while (chunk--)
    {
      high_byte = (crc >>> 24) & 0xFF;
      crc ^= crc32_table[high_lookup[high_byte]];
      crc <<= 8;
      crc += high_lookup[high_byte] ^ peek[chunk];
    }

  }

  return crc >>> 0;
}

function calc_span(prior_crc, post_crc)
{
  for (index = 3; index >= 0; index--)
  {
    table = (table << 8) | high_lookup[post_crc >>> 24];
    post_crc = (post_crc ^ crc32_table[table & 0xFF]) << 8;
  }
  for (index = 0; index < 4; index++)
  {
    byte_value = (prior_crc ^ table) & 0xFF;
    prior_crc = (prior_crc >>> 8) ^ crc32_table[table & 0xFF];
    span >>>= 8;
    span |= byte_value << 24;
    table >>>= 8;
  }

  return span;
}

function LinkPatch(filename)
{
  /* create a backup duplicate of the original ELF */
  backupFilename = filename + ".prepatch";
  CWSys.removeFile(backupFilename);
  CWSys.renameFile(filename, backupFilename);
  ElfFile.load(backupFilename);

  start = ElfFile.symbolValue("__vectors_start__");
  end   = ElfFile.symbolValue("__FLASH_segment_used_end__");

  /*
  compute size rounded up to whole 32-bit words
  (since this is what the DSU CRC32 peripheral uses)
  */
  size = end - start;
  if (size & 3)
    size += 4 - (size & 3);

  /* store app length within application itself */
  ElfFile.pokeUint32(start + app_len_offset, size);

  /*
  perform the calculation to determine what 32-bit value to write 
  in order to cause the overall CRC32 to calculate to be zero
  */
  pre_crc = crc32_calc(0xFFFFFFFF, start, app_crc_offset);
  latter_offset = app_crc_offset + 4;
  post_crc = reverse_crc32_calc(0 /* desired end value */, start + latter_offset, size - latter_offset);
  span = calc_span(pre_crc, post_crc);

  /* store app CRC within application itself */
  ElfFile.pokeUint32(start + app_crc_offset, span);

  /* write the modified data to the original filename */
  ElfFile.save(filename);
}
