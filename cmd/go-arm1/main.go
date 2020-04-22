package main

import (
	"io/ioutil"
	"os"

	"../../pkg/cpu"
	"../../pkg/elf"
	"../../pkg/memory"
)

func main() {
	filename := os.Args[1]
	file, err := os.Open(filename)
	defer file.Close()
	if err != nil {
		panic(err)
	}
	f, err := ioutil.ReadAll(file)
	elfd, err := elf.Initialize(&f)
	if err != nil {
		panic(err)
	}
	var rqMax uint32
	for _, seg := range elfd.Pheader {
		if rqMax < seg.VAddr+seg.MemSz {
			rqMax = seg.VAddr + seg.MemSz
		}
	}
	mem := memory.Initialize(rqMax * 0x10)
	/*for i := uint32(0); i < rqMax*0x10; i++ {
		mem.Write(i, &[]byte{byte(0xff)})
	}*/
	var sp uint32
	for _, seg := range elfd.Pheader {
		sm := f[seg.Offset : seg.Offset+seg.MemSz]
		mem.Write(seg.VAddr, &sm)
		//todo this may not work
		if seg.Ptype == 0x6474e551 { // GNU_STACK
			sp = seg.VAddr
		}
	}
	sp = 0x178000
	cp := cpu.CPU{}
	elfd.Entry = 0x103d0
	cp.Initialize([17]uint32{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sp, 0, elfd.Entry, 0}, mem)
	cp.Exec(mem)
}
