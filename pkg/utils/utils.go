package utils

func GetBits(v uint32, from, to uint8) (ret uint32) {
	ret = (v >> from) & ((1 << (to - from + 1)) - 1)
	return ret
}
func ArShiftR(v uint32, sh uint8) (ret uint32) {
	ret = uint32(int32(v) >> sh)
	return
}
func ArShiftL(v uint32, sh uint8) (ret uint32) {
	ret = uint32(int32(v) << sh)
	return
}
func LoShiftR(v uint32, sh uint8) (ret uint32) {
	ret = uint32(uint32(v) >> sh)
	return
}
func LoShiftL(v uint32, sh uint8) (ret uint32) {
	ret = uint32(uint32(v) << sh)
	return
}
