package cpu

import (
	"errors"
	"fmt"
	"math"
	"math/bits"

	"pkg/memory"
	"pkg/utils"
)

type CPU struct {
	Regs Registers
	mem  *memory.Memory
	Mode int
}
type Registers struct {
	register [17]uint32
}

func (cpu *CPU) Initialize(reg [17]uint32, m *memory.Memory) {
	cpu.Regs.register = reg
	cpu.mem = m
}
func (regs *Registers) getFlag(c rune) bool {
	var bit uint8
	switch c {
	case 'N':
		bit = 31
	case 'Z':
		bit = 30
	case 'C':
		bit = 29
	case 'V':
		bit = 28
	case 'I':
		bit = 7
	case 'F':
		bit = 6
	case 'T':
		bit = 5
	default:
		panic(errors.New(""))
	}
	return (regs.getReg(16)>>(bit-1))&0x1 == 1
}
func (regs *Registers) setFlag(c rune, b bool) {
	var bit uint8
	var a uint32
	if b {
		a = 1
	} else {
		a = 0
	}
	switch c {
	case 'N':
		bit = 31
	case 'Z':
		bit = 30
	case 'C':
		bit = 29
	case 'V':
		bit = 28
	case 'I':
		bit = 7
	case 'F':
		bit = 6
	case 'T':
		bit = 5
	default:
		panic(errors.New(""))
	}
	regs.setReg(16, regs.getReg(16) & ^uint32(0x1<<(bit-1)) + (a<<(bit-1)))
}
func (regs *Registers) setReg(n uint8, v uint32) {
	regs.register[n] = v
}
func (regs *Registers) getRawR15() uint32 {
	return regs.register[15]
}
func (regs *Registers) getReg(n uint8) uint32 {
	if n != 15 {
		return regs.register[n]
	} else {
		return regs.register[n] + 4
		//when pc is read, pc return the instruction address + 8byte. pc is already incremented after fetch, so +4byte is required.
	}
}
func (regs *Registers) incrementPC() {
	regs.register[15] += 4
}

/*
ModeBit | Mode | Accessible registers
0b10000 | User | PC, R14 to R0, CPSR
0b10001 | FIQ  | PC, R14_fiq to R8_fiq, R7 to R0, CPSR, SPSR_fiq
0b10010 | IRQ  | PC, R14_irq, R13_irq, R12 to R0, CPSR, SPSR_irq
0b10011 | Suvi | PC, R14_svc, R13_svc, R12 to R0, CPSR, SPSR_svc
0b10111 | Abort| PC, R14_abt, R13_abt, R12 to R0, CPSR, SPSR_abt
0b11011 | Udef | PC, R14_und, R13_und, R12 to R0, CPSR, SPSR_und
*/
func (regs *Registers) getMode(m uint8) uint8 {
	return uint8(regs.getReg(16) & 0x1f)
}
func (regs *Registers) setMode(m uint8) {
	regs.setReg(16, regs.getReg(16) & ^uint32(0x1f) + uint32(m&0x1f))
}

type InstructionArm32 struct {
	Cond        uint8 //4bit
	OpType      uint8 //3-4bit
	OpCode      uint8 //4bit
	Inst        uint32
	Condition   string
	Mnemonic    string
	Destination string
	Source      string
	Immediate   string
}

func (inst *InstructionArm32) getBits(from, to uint8) (ret uint32) {
	ret = getBits(inst.Inst, from, to)
	return ret
}
func (inst *InstructionArm32) getBit(addr uint8) (ret bool) {
	ret = inst.getBits(addr, addr) == 1
	return ret
}
func getBits(v uint32, from, to uint8) (ret uint32) {
	ret = (v >> from) & ((1 << (to - from + 1)) - 1)
	return ret
}
func getBit(v uint32, addr uint8) (ret bool) {
	ret = getBits(v, addr, addr) == 1
	return ret
}
func (inst *InstructionArm32) set(in uint32) error {
	//AND R1,R1,#0001 in little endian is
	//00000001 | 0001 0000 | 000) 0 0001 | 1110 001 0
	//Shift2     Rd   Shift1 op2  S Rn    cond ty op1
	inst.Inst = in
	inst.Cond = uint8(getBits(in, 28, 31))
	if inst.Cond == 0xf {
		return errors.New("condition 0b1111 is not supported")
	}
	inst.OpType = uint8(getBits(in, 25, 27))
	/*inst.Ibit = (in&0x2)>>0x1 == 1
	inst.Opcode = uint8((in&0x1)<<0x3 + (in&0xe000)>>0xd)
	inst.Sbit = (in&0x1000)>>0xc == 1
	inst.Rn = uint8((in & 0xf00) >> 0x8)
	inst.Rd = uint8((in & 0xf00000) >> 0x14)
	inst.ShifterOp = uint16((in&0xf0000)>>0x8 + (in&0xff000000)>>0x18)*/
	return nil
}

type InstructionThumb struct {
	Optype   uint8  //3bit
	Inst     uint16 //16bit
	Mnemonic string
}

func (inst *InstructionThumb) set(in uint16) {
	inst.Inst = in
	inst.Optype = uint8(getBits(uint32(inst.Inst), 13, 15))
}
func (cpu *CPU) checkCond(inst *InstructionArm32) (bool, string) {
	//Immediate Shift
	var condition bool
	var condname string
	switch inst.Cond {
	case 0x0:
		condition = cpu.Regs.getFlag('Z')
		condname = "EQ"
	case 0x1:
		condition = !cpu.Regs.getFlag('Z')
		condname = "NE"
	case 0x2:
		condition = cpu.Regs.getFlag('C')
		condname = "CS/HS"
	case 0x3:
		condition = !cpu.Regs.getFlag('C')
		condname = "CC/LO"
	case 0x4:
		condition = cpu.Regs.getFlag('N')
		condname = "MI"
	case 0x5:
		condition = !cpu.Regs.getFlag('N')
		condname = "PL"
	case 0x6:
		condition = cpu.Regs.getFlag('V')
		condname = "VS"
	case 0x7:
		condition = !cpu.Regs.getFlag('V')
		condname = "VC"
	case 0x8:
		condition = cpu.Regs.getFlag('C') && !cpu.Regs.getFlag('Z')
		condname = "HI"
	case 0x9:
		condition = !cpu.Regs.getFlag('C') && cpu.Regs.getFlag('Z')
		condname = "LS"
	case 0xa:
		condition = (cpu.Regs.getFlag('N') && cpu.Regs.getFlag('V')) || (!cpu.Regs.getFlag('N') && !cpu.Regs.getFlag('V'))
		condname = "GE"
	case 0xb:
		condition = (cpu.Regs.getFlag('N') && !cpu.Regs.getFlag('V')) || (!cpu.Regs.getFlag('N') && cpu.Regs.getFlag('V'))
		condname = "LT"
	case 0xc:
		condition = !cpu.Regs.getFlag('Z') && ((cpu.Regs.getFlag('N') && cpu.Regs.getFlag('V')) || (!cpu.Regs.getFlag('N') && !cpu.Regs.getFlag('V')))
		condname = "GT"
	case 0xd:
		condition = cpu.Regs.getFlag('Z') || ((cpu.Regs.getFlag('N') && !cpu.Regs.getFlag('V')) || (!cpu.Regs.getFlag('N') && cpu.Regs.getFlag('V')))
		condname = "LE"
	case 0xe:
		condition = true
		condname = "AL"
	case 0xf:
		panic(errors.New("undefined"))
	default:
		panic(errors.New("???"))
	}
	return condition, condname
}
func (cpu *CPU) execArm32(inst *InstructionArm32) error {
	cond, name := cpu.checkCond(inst)
	inst.Condition = name
	switch inst.OpType {
	case 0x0: //Data Processing
		if !cond {
			inst.Mnemonic = "\x1b[31mNOP\x1b[0m"
			return nil
		}
		if inst.getBits(23, 24) == 2 && inst.getBit(20) == false {
			//Miscellaneous instruction
            if inst.getBits(26,27)==0&&inst.getBits(23,24)==2&&inst.getBit(20)==false&&inst.getBits(28,31)!=0xf{
            //Control and DSP instruction space
            //MRS,MSR,BX,CLZ,BXJ,BLX,QADD,QDADD,QDSUB,BKPT,SMLA,SMLAW,SMULW,SMLAL,SMUL,MSR
            if inst.getBit(25)==false{
                if inst.getBit(4)==false{
                }else{

                }
            }
            }
			if inst.getBits(20, 27) == 22 /*0b00010110*/ && inst.getBits(4, 7) == 1 {
				//CLZ
				inst.Mnemonic = "CLZ"
				rm := uint8(inst.getBits(0, 3))
				vrm := cpu.Regs.getReg(rm)
				rd := uint8(inst.getBits(12, 15))
				vrd := uint32(bits.LeadingZeros32(vrm))
				cpu.Regs.setReg(rd, vrd)
				return nil
			}
			return errors.New("Miscellaneous instruction is not implemented")
		}
		shifter, _, shname := cpu.setDPShifter(inst)
		inst.Immediate = shname
		cpu.dpxs(inst, shifter)
	case 0x1: //Data Processing immediate
		var imm uint32
		imm = inst.getBits(0, 7)
		var rotImm int
		rotImm = int(inst.getBits(8, 11))
		shifter := bits.RotateLeft32(imm, -2*int(rotImm))
		shCarry := false
		if rotImm == 0 {
			shCarry = cpu.Regs.getFlag('C')
		} else {
			shCarry = getBits(shifter, 31, 31) == 1
		}
		cpu.Regs.setFlag('C', shCarry)
		inst.Immediate = fmt.Sprintf("#0x%x", shifter)
		if !cond {
			inst.Mnemonic = "\x1b[31mNOP\x1b[0m"
			return nil
		}
		cpu.dpxs(inst, shifter)
	case 0x2:
		fallthrough
	case 0x3:
		address, dn, sn := cpu.setLSAddr(inst)
		inst.Destination = dn
		inst.Source = sn
		cpu.doLS(inst, address)
	case 0x4: //Load&Store multiple instructions
		startAddr, endAddr := cpu.getLSMAddr(inst)
		fmt.Printf("start:end=%x:%x\n", startAddr, endAddr)
		err := cpu.doLSM(inst, startAddr, endAddr)
		if err != nil {
			return err
		}
	case 0x5: //B
		if !cond {
			inst.Mnemonic = "\x1b[31mNOP\x1b[0m"
			return nil
		}
		cpu.branch(inst)
	default:
		inst.Mnemonic = "UNDEFINED"
	}
	return nil
}
func (cpu *CPU) execThumb(inst *InstructionThumb) error {
	switch inst.Optype {
	case 7: //BL ,todo: sign extend
		inst.Mnemonic = "BL"
		h := getBits(uint32(inst.Inst), 11, 12)
		var offset uint32
		offset = getBits(uint32(inst.Inst), 0, 10)
		if h == 2 {
			cpu.Regs.setReg(14, cpu.Regs.getReg(14)+(offset<<1))
		} else if h == 3 {
			cpu.Regs.setReg(15, cpu.Regs.getReg(14)+(offset<<1))
			cpu.Regs.setReg(14, cpu.Regs.getReg(15)|1)
		} else if h == 0 {
			cpu.Regs.setReg(15, cpu.Regs.getReg(14)+(offset<<1)&0xfffffffc)
			cpu.Regs.setReg(14, cpu.Regs.getReg(15)|1)
			cpu.Regs.setFlag('T', false)
		}
	}
	return nil
}
func (cpu *CPU) setDPShifter(inst *InstructionArm32) (shifter uint32, shCarry bool, name string) {
	var rm uint8
	rm = uint8(inst.getBits(0, 3))
	var vrm uint32
	vrm = cpu.Regs.getReg(rm)
	var dptyp uint8
	dptyp = uint8(inst.getBits(4, 6))
	switch dptyp {
	case 0: //logical shift left by immediate
		var shiftImm uint8
		shiftImm = uint8(inst.getBits(7, 11))
		shifter = utils.LoShiftL(vrm, uint8(shiftImm))
		if shiftImm == 0 {
			shCarry = cpu.Regs.getFlag('C')
		} else {
			shCarry = getBits(vrm, 32-shiftImm, 32-shiftImm) == 1
		}
	case 1: //logical shift left by register
		var rs uint8
		rs = uint8(inst.getBits(8, 11))
		var vrs uint32
		vrs = cpu.Regs.getReg(rs)
		shifter = utils.LoShiftL(vrm, uint8(vrs))
		var vrss = uint8(getBits(vrs, 0, 7))
		if vrss == 0 {
			shCarry = cpu.Regs.getFlag('C')
		} else if vrss <= 32 {
			shCarry = getBits(vrm, 32-vrss, 32-vrss) == 1
		} else {
			shifter = 0
			shCarry = false
		}
	case 2: //logical shift right by immediate
		var shiftImm uint8
		shiftImm = uint8(inst.getBits(7, 11))
		shifter = utils.LoShiftR(vrm, uint8(shiftImm))
		if shiftImm == 0 {
			shCarry = getBits(vrm, 31, 31) == 1
		} else {
			shCarry = getBits(vrm, shiftImm-1, shiftImm-1) == 1
		}
	case 3: //logical shift right by register
		var rs uint8
		rs = uint8(inst.getBits(8, 11))
		var vrs uint32
		vrs = cpu.Regs.getReg(rs)
		shifter = utils.LoShiftR(vrm, uint8(vrs))
		var vrss = uint8(getBits(vrs, 0, 7))
		if vrss == 0 {
			shCarry = cpu.Regs.getFlag('C')
		} else if vrss <= 32 {
			shCarry = getBits(vrm, vrss-1, vrss-1) == 1
		} else {
			shifter = 0
			shCarry = false
		}
	case 4: //arithmetic shift right by immediate
		var shiftImm uint8
		shiftImm = uint8(inst.getBits(7, 11))
		if shiftImm == 0 {
			if getBits(vrm, 31, 31) == 0 {
				shifter = 0
				shCarry = getBits(vrm, 31, 31) == 1
			} else {
				shifter = 0xffffffff
				shCarry = getBits(vrm, 31, 31) == 1
			}
		} else {
			shifter = utils.ArShiftR(vrm, shiftImm)
			shCarry = getBits(vrm, shiftImm-1, shiftImm-1) == 1
		}
	case 5: //arithmetic shift right by register
		var rs uint8
		rs = uint8(inst.getBits(8, 11))
		var vrs uint32
		vrs = cpu.Regs.getReg(rs)
		shifter = utils.ArShiftR(vrm, uint8(getBits(vrs, 0, 3)))
		var vrss = uint8(getBits(vrs, 0, 7))
		if vrss == 0 {
			shCarry = cpu.Regs.getFlag('C')
		} else if vrss < 32 {
			shCarry = getBits(vrm, vrss-1, vrss-1) == 1
		} else {
			if getBits(vrm, 31, 31) == 0 {
				shifter = 0
				shCarry = getBits(vrm, 31, 31) == 1
			} else {
				shifter = 0xffffffff
				shCarry = getBits(vrm, 31, 31) == 1
			}
		}
	case 6: //rotate right by immediate
		var shiftImm uint8
		shiftImm = uint8(inst.getBits(7, 11))
		if shiftImm == 0 {
			shifter = (uint32(bool2uint8(cpu.Regs.getFlag('C'))) << 31) | (vrm >> 1)
			shCarry = getBits(vrm, 0, 0) == 1
		} else {
			shifter = bits.RotateLeft32(vrm, -1*int(shiftImm))
			shCarry = getBits(vrm, shiftImm-1, shiftImm-1) == 1
		}

	case 7: //rotate right by register
		var rs uint8
		rs = uint8(inst.getBits(8, 11))
		var vrs uint32
		vrs = cpu.Regs.getReg(rs)
		var vrss = uint8(getBits(vrs, 0, 7))
		if vrss == 0 {
			shifter = vrm
			shCarry = cpu.Regs.getFlag('C')
		} else if getBits(vrs, 0, 4) == 0 {
			shifter = vrm
			shCarry = getBits(vrm, 31, 31) == 1
		} else {
			shifter = bits.RotateLeft32(vrm, -1*int(getBits(vrs, 0, 4)))
			shCarry = getBits(vrm, uint8(getBits(vrs, 0, 4)-1), uint8(getBits(vrs, 0, 4))) == 1
		}
	}
	name = fmt.Sprintf("#0x%x", shifter)
	return shifter, shCarry, name
}
func (cpu *CPU) setLSAddr(inst *InstructionArm32) (address uint32, dn, sn string) {
	var rd uint8
	rd = uint8(inst.getBits(12, 15))
	var rn uint8
	rn = uint8(inst.getBits(16, 19))
	var vrn uint32
	vrn = cpu.Regs.getReg(rn)
	var ubit bool
	ubit = inst.getBits(23, 23) == 1
	switch inst.OpType {
	case 0x2:
		offset := inst.getBits(0, 11)
		switch inst.getBits(21, 21) {
		case 0: //immediate offset
			switch inst.getBits(24, 24) {
			case 0: //immediate post-indexed
				cond, _ := cpu.checkCond(inst)
				if cond {
					if ubit {
						sn = fmt.Sprintf("post{r%d+#0x%x}=%x", rn, offset, vrn+offset)
						cpu.Regs.setReg(rn, vrn+offset)
					} else {
						sn = fmt.Sprintf("post{r%d-#0x%x}=%x", rn, offset, vrn-offset)
						cpu.Regs.setReg(rn, vrn-offset)
					}
				}
			case 1:
				if ubit {
					sn = fmt.Sprintf("{r%d+#0x%x}=%x", rn, offset, vrn+offset)
					address = vrn + offset
				} else {
					sn = fmt.Sprintf("{r%d-#0x%x}=%x", rn, offset, vrn-offset)
					address = vrn - offset
				}
			}
		case 1:
			switch inst.getBits(24, 24) {
			case 0: //register pre-indexed
				rm := uint8(inst.getBits(0, 3))
				vrm := cpu.Regs.getReg(rm)
				if ubit {
					sn = fmt.Sprintf("{r%d+r%d}=%x", rn, rm, vrn+vrm)
					address = vrn + vrm
				} else {
					sn = fmt.Sprintf("{r%d-r%d}=%x", rn, rm, vrn-vrm)
					address = vrn - vrm
				}
				cond, _ := cpu.checkCond(inst)
				if cond {
					fmt.Println("upd")
					cpu.Regs.setReg(rn, address)
				}
			case 1: //immediate pre-indexed

				fmt.Printf("adddr:%x", cpu.Regs.getReg(15))
				if ubit {
					sn = fmt.Sprintf("{r%d+#0x%x}=%x", rn, offset, vrn+offset)
					address = vrn + offset
				} else {
					sn = fmt.Sprintf("{r%d-#0x%x}=%x", rn, offset, vrn-offset)
					address = vrn - offset
				}
				cond, _ := cpu.checkCond(inst)
				if cond {
					fmt.Println("update")
					cpu.Regs.setReg(rn, address)
				}

			}
		}
	case 0x3: //register offset
		if getBits(inst.Inst, 24, 24) != 1 || inst.getBits(21, 21) != 0 {
			panic(errors.New("unknown"))
		}
		rm := uint8(inst.getBits(0, 3))
		vrm := cpu.Regs.getReg(rm)
		if inst.getBits(4, 11) == 0 { //5.2.3
			if ubit {
				sn = fmt.Sprintf("{r%d+r%d}=%x", rn, rm, vrn+vrm)
				address = vrn + vrm
			} else {
				sn = fmt.Sprintf("{r%d-r%d}=%x", rn, rm, vrn-vrm)
				address = vrn - vrm
			}
		} else { //5.2.4
			var index uint32
			var shImm uint8
			shImm = uint8(inst.getBits(7, 11))
			switch inst.getBits(5, 6) {
			case 0x0: //LSL
				index = utils.LoShiftL(vrm, shImm)
			case 0x1: //LSR
				if shImm == 0 {
					index = 0
				} else {
					index = utils.LoShiftR(vrm, shImm)
				}
			case 0x2:
				if shImm == 0 {
					if getBits(vrm, 31, 31) == 1 {
						index = 0xffffffff
					} else {
						index = 0xffffffff
					}
				} else {
					index = utils.ArShiftR(vrm, shImm)
				}
			case 0x3:
				if shImm == 0 {
					index = utils.LoShiftL(uint32(bool2uint8(cpu.Regs.getFlag('C'))), 31) | utils.LoShiftR(vrm, shImm)
				} else {
					index = bits.RotateLeft32(vrm, -1*int(shImm))
				}
			}
			if ubit {
				address = vrn + index
			} else {
				address = vrn - index
			}
		}
	}
	dn = fmt.Sprintf("r%d", rd)
	return address, dn, sn
}
func (cpu *CPU) dpxs(inst *InstructionArm32, shifter uint32) error {
	var rn, rd uint8
	//fmt.Printf("%x\n", inst.getBits(0, 31))
	rn = uint8(inst.getBits(16, 19))
	rd = uint8(inst.getBits(12, 15))
	var vrd uint32
	vrd = cpu.Regs.getReg(rd)
	var n, z, c, v bool
	n = cpu.Regs.getFlag('N')
	z = cpu.Regs.getFlag('Z')
	c = cpu.Regs.getFlag('C')
	v = cpu.Regs.getFlag('V')
	var s bool
	s = inst.getBits(20, 20) == 1
	inst.OpCode = uint8(inst.getBits(21, 24))
	inst.Source = fmt.Sprintf("r%d", rn)
	inst.Destination = fmt.Sprintf("r%d", rd)
	switch inst.OpCode {
	case 0x0: //AND:logical and
		//4.1.4, 3.13: extending the instruction set. this isn't AND instruction
		if inst.getBit(25) == false && (inst.getBit(4) == true && inst.getBit(7) == true) {
			//3.13.1 undefined
			if inst.getBits(25, 27) == 3 && inst.getBit(4) == true {
				panic(errors.New("UNDEFINED"))
			}
			//3.13.2 arithmetic instruction extension space
			if inst.getBits(24, 27) == 0 && inst.getBits(4, 7) == 9 && inst.getBits(28, 31) != 15 {
				opc := inst.getBits(21, 23)
				switch opc {
				//todo: assembly of MUL is't implemented adequately
				case 0: //MUL,MULS
					inst.Mnemonic = "MUL"
					vrm := cpu.Regs.getReg(uint8(inst.getBits(0, 3)))
					vrs := cpu.Regs.getReg(uint8(inst.getBits(8, 11)))
					rd = uint8(inst.getBits(16, 19))
					vrd = vrm * vrs
					if s {
						n = getBit(vrd, 31)
						z = vrd == 0
					}
				}
			}
		} else {
			inst.Mnemonic = "AND"
			vrd = cpu.Regs.getReg(rn) & shifter
			if s {
				n = int32(vrd) < 0 //あってる？
				z = vrd == 0
			}
		}
	case 0x1: //EOR:logical eor
		inst.Mnemonic = "EOR"
		vrd = cpu.Regs.getReg(rn) ^ shifter
		if s {
			n = int32(vrd) < 0 //あってる？
			z = vrd == 0
		}
	case 0x2: //SUB:sub
		inst.Mnemonic = "SUB"
		vrd = cpu.Regs.getReg(rn) - shifter
		if s {
			n = int32(vrd) < 0 //あってる？
			z = vrd == 0
			//todo implement c,v flag manup
		}
	case 0x3: //RSB:reverse sub
		inst.Mnemonic = "RSB"
		vrd = shifter - cpu.Regs.getReg(rn)
		if s {
			n = int32(vrd) < 0 //あってる？
			z = vrd == 0
			//todo
		}
	case 0x4: //ADD:add
		inst.Mnemonic = "ADD"
		res := uint64(cpu.Regs.getReg(rn)) + uint64(shifter)
		vrd = uint32(res)
		if s {
			n = int32(vrd) < 0 //あってる？
			z = vrd == 0
			c = res > math.MaxUint32
			v = res > math.MaxInt32
		}
	case 0x5: //ADC:add with carry
		inst.Mnemonic = "ADC"
		res := uint64(cpu.Regs.getReg(rn)) + uint64(shifter) + uint64(bool2uint8(cpu.Regs.getFlag('C')))
		vrd = uint32(res)
		if s {
			n = int32(vrd) < 0 //あってる？
			z = vrd == 0
			c = res > math.MaxUint32
			v = res > math.MaxInt32
		}

	case 0x6: //SBC:sub with carry
		inst.Mnemonic = "SBC"
		vrd = cpu.Regs.getReg(rn) - shifter - uint32(bool2uint8(!cpu.Regs.getFlag('C')))
		if s {
			n = int32(vrd) < 0 //あってる？
			z = vrd == 0
			//todo
		}
	case 0x7: //RSC:reverse sub with carry
		inst.Mnemonic = "RSC"
		vrd = shifter - cpu.Regs.getReg(rn) - uint32(bool2uint8(!cpu.Regs.getFlag('C')))
		if s {
			n = int32(vrd) < 0 //あってる？
			z = vrd == 0
			//todo
		}
	case 0x8: //TST:test
		inst.Mnemonic = "TST"
		res := cpu.Regs.getReg(rn) & shifter
		n = int32(vrd) < 0 //あってる？
		z = res == 0
	case 0x9: //TEQ:test equivalence
		inst.Mnemonic = "TEQ"
		res := cpu.Regs.getReg(rn) ^ shifter
		n = int32(vrd) < 0 //あってる？
		z = res == 0
	case 0xa: //CMP:compare
		inst.Mnemonic = "CMP"
		res := cpu.Regs.getReg(rn) - shifter
		n = int32(vrd) < 0 //あってる？
		z = res == 0
		//todo
	case 0xb: //CMN:compare negated
		inst.Mnemonic = "CMN"
		res := int64(cpu.Regs.getReg(rn)) + int64(shifter)
		n = int32(vrd) < 0 //あってる？
		z = res == 0
		c = res > math.MaxUint32
		v = res > math.MaxInt32
	case 0xc: //ORR:logical or
		inst.Mnemonic = "ORR"
		vrd = cpu.Regs.getReg(rn) | shifter
		if s {
			n = int32(vrd) < 0
			z = vrd == 0
		}
	case 0xd: //MOV:move
		inst.Mnemonic = "MOV"
		vrd = shifter
		if s {
			n = int32(vrd) < 0
			z = vrd == 0
		}
	case 0xe: //BIC:bit clear
		inst.Mnemonic = "BIC"
		vrd = cpu.Regs.getReg(rn) & ^shifter
		if s {
			n = int32(vrd) < 0
			z = vrd == 0
		}
	case 0xf: //MVN:move not
		inst.Mnemonic = "MVN"
		vrd = ^shifter
		if s {
			n = int32(vrd) < 0
			z = vrd == 0
		}
	}
	cpu.Regs.setReg(rd, vrd)
	cpu.Regs.setFlag('N', n)
	cpu.Regs.setFlag('Z', z)
	cpu.Regs.setFlag('C', c)
	cpu.Regs.setFlag('V', v)
	return nil
}

func (cpu *CPU) getLSMAddr(inst *InstructionArm32) (startAddr, endAddr uint32) {
	//See A5.4 Addressing Mode 4- Load and Store Multiple
	var rn uint8
	rn = uint8(inst.getBits(16, 19))
	var vrn uint32
	vrn = cpu.Regs.getReg(rn)
	cond, _ := cpu.checkCond(inst)
	numberOfBitsx4 := uint32(4 * bits.OnesCount32(inst.getBits(0, 15)))
	if inst.getBit(23) {
		//Ubit is 1
		if !inst.getBit(24) {
			//Pbit is 0, Increment after
			startAddr = vrn
			endAddr = vrn + numberOfBitsx4 - 4
			if cond && inst.getBit(21) {
				vrn = vrn + numberOfBitsx4
			}
		} else {
			//Increment before
			startAddr = vrn + 4
			endAddr = vrn + numberOfBitsx4
			if cond && inst.getBit(21) {
				vrn = vrn + numberOfBitsx4
			}
		}
	} else {
		if !inst.getBit(24) {
			//Decrement after
			startAddr = vrn - numberOfBitsx4 + 4
			endAddr = vrn
			if cond && inst.getBit(21) {
				vrn = vrn - numberOfBitsx4
			}
		} else {
			//Decrement before
			startAddr = vrn - numberOfBitsx4
			endAddr = vrn - 4
			if cond && inst.getBit(21) {
				vrn = vrn - numberOfBitsx4
			}
		}
	}
	cpu.Regs.setReg(rn, vrn)
	return startAddr, endAddr
}

func (cpu *CPU) doLSM(inst *InstructionArm32, startAddr, endAddr uint32) error {
	//todo: what is meomoryaccess(b-bit,e-bit)????
	//todo LDM(2),LDM(3) isn't implemented
	cond, _ := cpu.checkCond(inst)
	if !cond {
		return nil
	}
	if inst.getBit(20) {
		//Load
		inst.Mnemonic = "LDM"
		addr := startAddr
		for i := uint8(0); i < 14; i++ {
			if inst.getBit(i) {
				cpu.Regs.setReg(i, cpu.mem.Fetch32bits(addr))
				addr += 4
			}
		}
		if inst.getBit(15) {
			value := cpu.mem.Fetch32bits(addr)
			cpu.Regs.setReg(15, value&0xFFFFFFFE)
			cpu.Regs.setFlag('T', value&1 == 1)
			addr = addr + 4
		}
		if endAddr != addr-4 {
			return errors.New("startAddr and endAddr is mismatched")
		}
	} else {
		inst.Mnemonic = "STM"
		inst.Destination = "{"
		addr := startAddr
		for i := uint8(0); i < 15; i++ {
			if inst.getBit(i) {
				cpu.mem.Write32bits(addr, cpu.Regs.getReg(i))
				addr = addr + 4
				inst.Destination += fmt.Sprintf("r%d, ", i)
				//Shared is not implemented
			}
		}
		inst.Destination += "}"
		if endAddr != addr-4 {
			return errors.New("startAddr and endAddr is mismatched")
		}
	}
	return nil
}

func (cpu *CPU) doMS(inst *InstructionArm32) error{
    
    return nil
}
func (cpu *CPU) doLS(inst *InstructionArm32, addr uint32) error { //store multiple isn't defined
	var rd uint8
	rd = uint8(inst.getBits(12, 15))
	var vrd uint32
	vrd = cpu.Regs.getReg(rd)
	cond, _ := cpu.checkCond(inst)
	if !cond {
		return nil
	}
	switch inst.getBits(20, 20) {
	case 0:
		switch inst.getBits(22, 22) {
		case 0: //word
			inst.Mnemonic = "STR"
			cpu.mem.Write32bits(addr, uint32(vrd))
		case 1: //byte
			inst.Mnemonic = "STRB"
			b := []byte{byte(vrd)}
			cpu.mem.Write(addr, &b)
		}
	case 1:
		switch inst.getBits(22, 22) {
		case 0:
			inst.Mnemonic = "LDR"
			var v, m uint32
			m = cpu.mem.Fetch32bits(addr + 4)
			//bug!!!!!! why is +4 required?????????

			switch addr & 3 {
			case 0:
				v = m
			case 1:
				v = bits.RotateLeft32(m, -8)
			case 2:
				v = bits.RotateLeft32(m, -16)
			case 3:
				v = bits.RotateLeft32(m, -24)
			}
			if rd == 15 {
				panic(errors.New(""))
			} else {
				cpu.Regs.setReg(rd, v)
			}
		case 1:
			inst.Mnemonic = "LDRB"
			cpu.Regs.setReg(rd, uint32(cpu.mem.Fetch32bits(addr)))
		}
	}
	return nil
}
func (cpu *CPU) branch(inst *InstructionArm32) error {
	inst.Mnemonic = "BL"
	cond, _ := cpu.checkCond(inst)
	if !cond {
		return nil
	}
	if inst.getBit(24) == true {
		cpu.Regs.setReg(14, cpu.Regs.getReg(15))
	}
	var off uint32
	if inst.getBit(23) == true { //sign
		off = inst.getBits(0, 23) | 0xff000000
	} else {
		off = inst.getBits(0, 23)
	}
	off = off << 2
	cpu.Regs.setReg(15, uint32(cpu.Regs.getReg(15)+off))
	return nil
}
func bool2uint8(b bool) uint8 {
	if b {
		return 1
	} else {
		return 0
	}
}

func (cpu *CPU) Start() {
	for i := 0; i < 50; i++ {
		if cpu.Regs.getReg(15)&1 == 1 { //thumb
			i := cpu.mem.Fetch16bits(cpu.Regs.getRawR15() &^ 1)
			inst := &InstructionThumb{}
			inst.set(uint16(i))
			cpu.execThumb(inst)
			fmt.Printf("%s:%x,%x\n", inst.Mnemonic, cpu.Regs.getReg(15), inst.Inst)
			//bug
			cpu.Regs.incrementPC()
		} else { //arm32
			i := cpu.mem.Fetch32bits(cpu.Regs.getRawR15() &^ uint32(1))
			cpu.Regs.incrementPC()

			inst := &InstructionArm32{}
			inst.set(i)
			//fmt.Println(inst.Cond)
			err := cpu.execArm32(inst)
			if err != nil {
				fmt.Println(err)
				cpu.dumpDebugInfo(inst)
				return
			}
			fmt.Printf("%x:%x:%s.%s, %s, %s, %s\n", cpu.Regs.getReg(15), inst.Inst, inst.Mnemonic, inst.Condition, inst.Destination, inst.Source, inst.Immediate)
			if inst.Mnemonic == "" {
				fmt.Print("\x1b[31m!!!!UNDEFINED!!!!\x1b[0m\n")
				cpu.dumpDebugInfo(inst)
				return
			}
		}
		if i == 6 {
			cpu.mem.Dump()
			fmt.Println("file write ok.")
		}
		cpu.dumpRegisters()
		//fmt.Printf("flags N:%t, Z:%t, C:%t, V:%t \n\n", cpu.Regs.getFlag('N'), cpu.Regs.getFlag('Z'), cpu.Regs.getFlag('C'), cpu.Regs.getFlag('V'))
	}
}
func (cpu *CPU) dumpDebugInfo(inst *InstructionArm32) {
	var instBinary string
	for i := 1; i < 33; i++ {
		if (inst.Inst>>(32-i))&1 == 1 {
			instBinary += "1"
		} else {
			instBinary += "0"
		}
		if i%4 == 0 {
			if i%8 != 0 {
				instBinary += " "
			} else {
				instBinary += " | "
			}
		}
	}
	fmt.Println(instBinary)
	fmt.Printf("%x:%x:%s.%s, %s, %s, %s\n", cpu.Regs.getReg(15), inst.Inst, inst.Mnemonic, inst.Condition, inst.Destination, inst.Source, inst.Immediate)
	cpu.dumpRegisters()
	fmt.Printf("flags N:%t, Z:%t, C:%t, V:%t \n\n", cpu.Regs.getFlag('N'), cpu.Regs.getFlag('Z'), cpu.Regs.getFlag('C'), cpu.Regs.getFlag('V'))
}
func (cpu *CPU) dumpRegisters() {
	for i := uint8(0); i < 4; i++ {
		fmt.Printf("r%2d:%8x r%2d:%8x r%2d:%8x r%2d:%8x \n",
			i*4, cpu.Regs.getReg(i*4), i*4+1, cpu.Regs.getReg(i*4+1), i*4+2, cpu.Regs.getReg(i*4+2), i*4+3, cpu.Regs.getReg(i*4+3))
	}
}

/*
func a() {
	filename := os.Args[1]
	file, err := os.Open(filename)
	defer file.Close()
	if err != nil {
		panic(err)
	}
	memory := new([]byte)
	exefile, err := ioutil.ReadAll(file)
	var eof uint32
	eof = uint32(len(exefile))
	memory = &exefile
	elf.Set(memory)
	inst := Instruction{}
	cpu := CPU{}
	for ; cpu.Regs.getReg(15) < eof; cpu.Regs.setReg(15, cpu.Regs.getReg(15)+4) {
		var tmp uint32
		tmp = binary.LittleEndian.Uint32((*memory)[cpu.Regs.getReg(15) : cpu.Regs.getReg(15)+4])
		inst.set(tmp)
		cpu.exec(&inst)
		for j, v := range cpu.Regs.register {
			if 4 < j && j < 14 {
				continue
			}
			fmt.Printf("r%d:%x\n", j, v)
		}
		fmt.Printf("flags N:%t, Z:%t, C:%t, V:%t \n\n", cpu.Regs.getFlag('N'), cpu.Regs.getFlag('Z'), cpu.Regs.getFlag('C'), cpu.Regs.getFlag('V'))
	}
}*/
