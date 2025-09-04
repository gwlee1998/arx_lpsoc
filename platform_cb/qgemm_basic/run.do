quit -sim
vdel -lib work -all
vlib work
vmap work work
vlog +acc tb/tb_qdq_controller.sv tb/tb_quantize_array.sv rtl/*.v
vsim -gui -voptargs="+acc" work.tb_qdq_controller
run -all