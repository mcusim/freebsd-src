PROG_CXX=	llvm-tblgen
MAN=

CFLAGS+=	-I${.OBJDIR}

SRCDIR=		llvm/utils/TableGen
SRCS+=		AsmMatcherEmitter.cpp
SRCS+=		AsmWriterEmitter.cpp
SRCS+=		AsmWriterInst.cpp
SRCS+=		Attributes.cpp
SRCS+=		CTagsEmitter.cpp
SRCS+=		CallingConvEmitter.cpp
SRCS+=		CodeEmitterGen.cpp
SRCS+=		CodeGenDAGPatterns.cpp
SRCS+=		CodeGenHwModes.cpp
SRCS+=		CodeGenInstAlias.cpp
SRCS+=		CodeGenInstruction.cpp
SRCS+=		CodeGenIntrinsics.cpp
SRCS+=		CodeGenMapTable.cpp
SRCS+=		CodeGenRegisters.cpp
SRCS+=		CodeGenSchedule.cpp
SRCS+=		CodeGenTarget.cpp
SRCS+=		CompressInstEmitter.cpp
SRCS+=		DAGISelEmitter.cpp
SRCS+=		DAGISelMatcher.cpp
SRCS+=		DAGISelMatcherEmitter.cpp
SRCS+=		DAGISelMatcherGen.cpp
SRCS+=		DAGISelMatcherOpt.cpp
SRCS+=		DFAEmitter.cpp
SRCS+=		DFAPacketizerEmitter.cpp
SRCS+=		DXILEmitter.cpp
SRCS+=		DecoderEmitter.cpp
SRCS+=		DirectiveEmitter.cpp
SRCS+=		DisassemblerEmitter.cpp
SRCS+=		ExegesisEmitter.cpp
SRCS+=		FastISelEmitter.cpp
SRCS+=		GlobalISel/CXXPredicates.cpp
SRCS+=		GlobalISel/CodeExpander.cpp
SRCS+=		GlobalISel/MatchDataInfo.cpp
SRCS+=		GlobalISel/Patterns.cpp
SRCS+=		GlobalISelCombinerEmitter.cpp
SRCS+=		GlobalISelEmitter.cpp
SRCS+=		GlobalISelMatchTable.cpp
SRCS+=		GlobalISelMatchTableExecutorEmitter.cpp
SRCS+=		InfoByHwMode.cpp
SRCS+=		InstrDocsEmitter.cpp
SRCS+=		InstrInfoEmitter.cpp
SRCS+=		IntrinsicEmitter.cpp
SRCS+=		MacroFusionPredicatorEmitter.cpp
SRCS+=		OptEmitter.cpp
SRCS+=		OptParserEmitter.cpp
SRCS+=		OptRSTEmitter.cpp
SRCS+=		PredicateExpander.cpp
SRCS+=		PseudoLoweringEmitter.cpp
SRCS+=		RISCVTargetDefEmitter.cpp
SRCS+=		RegisterBankEmitter.cpp
SRCS+=		RegisterInfoEmitter.cpp
SRCS+=		SDNodeProperties.cpp
SRCS+=		SearchableTableEmitter.cpp
SRCS+=		SubtargetEmitter.cpp
SRCS+=		SubtargetFeatureInfo.cpp
SRCS+=		TableGen.cpp
SRCS+=		Types.cpp
SRCS+=		VTEmitter.cpp
SRCS+=		VarLenCodeEmitterGen.cpp
SRCS+=		WebAssemblyDisassemblerEmitter.cpp
SRCS+=		X86CompressEVEXTablesEmitter.cpp
SRCS+=		X86DisassemblerTables.cpp
SRCS+=		X86FoldTablesEmitter.cpp
SRCS+=		X86MnemonicTables.cpp
SRCS+=		X86ModRMFilters.cpp
SRCS+=		X86RecognizableInstr.cpp

.include "${SRCTOP}/lib/clang/llvm.pre.mk"

llvm/CodeGen/GenVT.inc: ${LLVM_SRCS}/include/llvm/CodeGen/ValueTypes.td
	${LLVM_MIN_TBLGEN} -gen-vt \
	    -I ${LLVM_SRCS}/include -d ${.TARGET}.d -o ${.TARGET} \
	    ${LLVM_SRCS}/include/llvm/CodeGen/ValueTypes.td
TGHDRS+=	llvm/CodeGen/GenVT.inc

DEPENDFILES+=	${TGHDRS:C/$/.d/}
DPSRCS+=	${TGHDRS}
CLEANFILES+=	${TGHDRS} ${TGHDRS:C/$/.d/}
CLEANFILES+=	${GENSRCS} ${GENSRCS:C/$/.d/}

.include "../llvm.prog.mk"
