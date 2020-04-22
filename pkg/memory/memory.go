package memory

import (
	"encoding/binary"
)

type Memory struct {
	mem []byte
}

func Initialize(size uint32) *Memory {
	b := make([]byte, size)
	mem := Memory{mem: b}
	return &mem
}
func (memory *Memory) Write(off uint32, by *[]byte) {
	for i, v := range *by {
		memory.mem[uint32(i)+off] = v
	}
}
func (memory *Memory) Write32bits(off uint32, by uint32) {
	for i := 0; i < 4; i++ {
		memory.mem[off+uint32(i)] = byte((by >>(i*8)) & 0xff)
	}
}
func (memory *Memory) Fetch32bits(from uint32) (ret uint32) {
	return binary.LittleEndian.Uint32(((*memory).mem)[from : from+4])
}
func (memory *Memory) Fetch16bits(from uint32) (ret uint16) {
	return binary.LittleEndian.Uint16(((*memory).mem)[from : from+2])
}
func (memory *Memory) Fetchbyte(from uint32) (ret byte) {
	return memory.mem[from]
}
func (memory *Memory) Fetch(from, size uint32) (ret *[]byte) {
	re := memory.mem[from : from+size-1]
	return &re
}
