cd C:/Users/cc891/qgemm_basic

catch {vdel -lib work -all}
vlib work
vmap work work

# 필요 파일만 한 번에 컴파일 (eval: 와일드카드 확실히 펼치기)
eval vlog -sv +acc +incdir+rtl +incdir+tb -work work rtl/*.v tb/*.sv

# TB만 로드. 여기서 멈춤(자동 실행 안 함)
vsim work.tb_qdq_controller