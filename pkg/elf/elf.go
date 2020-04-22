package elf

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
)

type ElfHeader32 struct {
	Ident     ElfIdent
	Typ       uint16
	Machine   uint16
	Version   uint32
	Entry     uint32
	Phoff     uint32
	Shoff     uint32
	Flags     uint32
	Ehsize    uint16
	Phentsize uint16
	Phnum     uint16
	Shentsize uint16
	Shnum     uint16
	Shstrndx  uint16

	Pheader []ElfPHeader32
}
type ElfIdent struct {
	Mag     uint32 //magic number 0x7f ELF
	Class   byte   //1->32bit,2->64bit
	Data    byte   //1->little endian,2-> big endian
	Version byte   //1
	Osabi   byte   //0x0 indicates Unix System V
	Abiver  byte
	Pad     [7]byte //unused
}
type ElfPHeader32 struct {
	Ptype  uint32
	Offset uint32
	VAddr  uint32
	PAddr  uint32
	FileSz uint32
	MemSz  uint32
	Flags  uint32
	Align  uint32
}

func checkElf(mem [4]byte) bool {
	if mem[1] == 'E' && mem[2] == 'L' && mem[3] == 'F' {
		return true
	}
	return false
}
func Initialize(mem *[]byte) (*ElfHeader32, error) {
	header := (*mem)[0:0x200]
	head := ElfHeader32{}
	head.Ident.Mag = binary.BigEndian.Uint32(header[0x0:0x4])
	head.Ident.Class = header[0x4]
	head.Ident.Data = header[0x5]
	head.Ident.Version = header[0x6]
	head.Ident.Osabi = header[0x7]
	head.Ident.Abiver = header[0x8]
	if head.Ident.Class != 0x01 {
		return nil, errors.New("64bit isn't supported")
	}

	switch head.Ident.Data {
	case 0x1: //little endian
		head.Typ = binary.LittleEndian.Uint16(header[0x10:0x12])
		head.Machine = binary.LittleEndian.Uint16(header[0x12:0x14])
		head.Version = binary.LittleEndian.Uint32(header[0x14:0x18])
		head.Entry = binary.LittleEndian.Uint32(header[0x18:0x1c])
		head.Phoff = binary.LittleEndian.Uint32(header[0x1c:0x20])
		head.Shoff = binary.LittleEndian.Uint32(header[0x20:0x24])
		head.Flags = binary.LittleEndian.Uint32(header[0x24:0x28])
		head.Ehsize = binary.LittleEndian.Uint16(header[0x28:0x2a])
		head.Phentsize = binary.LittleEndian.Uint16(header[0x2a:0x2c])
		head.Phnum = binary.LittleEndian.Uint16(header[0x2c:0x2e])
		head.Shentsize = binary.LittleEndian.Uint16(header[0x2e:0x30])
		head.Shnum = binary.LittleEndian.Uint16(header[0x30:0x32])
		head.Shstrndx = binary.LittleEndian.Uint16(header[0x32:0x34])
	case 0x2: //big endian
		return nil, errors.New("big endian isn't supported")
	}
	if head.Machine != 0x28 {
		return nil, errors.New("Unknown architecture, Please recompile for ARM")
	}
	fmt.Println(head)
	if head.Phoff == 0 {
		return &head, nil
	}
	var poff uint32
	poff = head.Phoff
	for i := uint16(0); i < head.Phnum; i++ {
		phead := ElfPHeader32{}
		reader := bytes.NewReader(header[poff : poff+uint32(head.Phentsize)])
		binary.Read(reader, binary.LittleEndian, &phead)
		poff = poff + uint32(head.Phentsize)
		head.Pheader = append(head.Pheader, phead)
	}
	return &head, nil
}
