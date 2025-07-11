set(BERRY_SRC_C
	${BERRY_SRCPATH}/be_api.c
	${BERRY_SRCPATH}/be_baselib.c
	${BERRY_SRCPATH}/be_bytecode.c
	${BERRY_SRCPATH}/be_byteslib.c
	${BERRY_SRCPATH}/be_class.c
	${BERRY_SRCPATH}/be_code.c
	${BERRY_SRCPATH}/be_debug.c
	${BERRY_SRCPATH}/be_debuglib.c
	${BERRY_SRCPATH}/be_exec.c
	${BERRY_SRCPATH}/be_filelib.c
	${BERRY_SRCPATH}/be_func.c
	${BERRY_SRCPATH}/be_gc.c
	${BERRY_SRCPATH}/be_gclib.c
	${BERRY_SRCPATH}/be_globallib.c
	${BERRY_SRCPATH}/be_introspectlib.c
	${BERRY_SRCPATH}/be_jsonlib.c
	${BERRY_SRCPATH}/be_lexer.c
	${BERRY_SRCPATH}/be_libs.c
	${BERRY_SRCPATH}/be_list.c
	${BERRY_SRCPATH}/be_listlib.c
	${BERRY_SRCPATH}/be_map.c
	${BERRY_SRCPATH}/be_maplib.c
	${BERRY_SRCPATH}/be_mathlib.c
	${BERRY_SRCPATH}/be_mem.c
	${BERRY_SRCPATH}/be_module.c
	${BERRY_SRCPATH}/be_object.c
	${BERRY_SRCPATH}/be_oslib.c
	${BERRY_SRCPATH}/be_parser.c
	${BERRY_SRCPATH}/be_rangelib.c
	${BERRY_SRCPATH}/be_repl.c
	${BERRY_SRCPATH}/be_solidifylib.c
	${BERRY_SRCPATH}/be_strictlib.c
	${BERRY_SRCPATH}/be_string.c
	${BERRY_SRCPATH}/be_strlib.c
	${BERRY_SRCPATH}/be_syslib.c
	${BERRY_SRCPATH}/be_timelib.c
	${BERRY_SRCPATH}/be_undefinedlib.c
	${BERRY_SRCPATH}/be_var.c
	${BERRY_SRCPATH}/be_vector.c
	${BERRY_SRCPATH}/be_vm.c

	${BERRY_MODULEPATH}/../be_bindings.c
	${BERRY_MODULEPATH}/../be_modtab.c
	${BERRY_MODULEPATH}/../be_port.c
	${BERRY_MODULEPATH}/../be_run.c
	
	${BERRY_MODULEPATH}/be_i2c.c
)
