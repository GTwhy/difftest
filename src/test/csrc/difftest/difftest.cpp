/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

#include "difftest.h"
#include "goldenmem.h"
#include "ram.h"
#include "flash.h"
#include "spikedasm.h"
#include "ref.h"
#include "tools.h"
#include <vector>

static const char *reg_name[DIFFTEST_NR_REG+1] = {
  "$0",  "ra",  "sp",   "gp",   "tp",  "t0",  "t1",   "t2",
  "s0",  "s1",  "a0",   "a1",   "a2",  "a3",  "a4",   "a5",
  "a6",  "a7",  "s2",   "s3",   "s4",  "s5",  "s6",   "s7",
  "s8",  "s9",  "s10",  "s11",  "t3",  "t4",  "t5",   "t6",
  "ft0", "ft1", "ft2",  "ft3",  "ft4", "ft5", "ft6",  "ft7",
  "fs0", "fs1", "fa0",  "fa1",  "fa2", "fa3", "fa4",  "fa5",
  "fa6", "fa7", "fs2",  "fs3",  "fs4", "fs5", "fs6",  "fs7",
  "fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11",
  "this_pc",
  "mstatus", "mcause", "mepc",
  "sstatus", "scause", "sepc",
  "satp",
  "mip", "mie", "mscratch", "sscratch", "mideleg", "medeleg",
  "mtval", "stval", "mtvec", "stvec", "mode",
#ifdef DEBUG_MODE_DIFF
  "debug mode", "dcsr", "dpc", "dscratch0", "dscratch1",
 #endif
};

Difftest **difftest = NULL;

int difftest_init() {
  difftest = new Difftest*[NUM_CORES];
  for (int i = 0; i < NUM_CORES; i++) {
    difftest[i] = new Difftest(i);
  }
  mcm_checker_init();
  return 0;
}

int init_nemuproxy(size_t ramsize = 0) {
  for (int i = 0; i < NUM_CORES; i++) {
    difftest[i]->update_nemuproxy(i, ramsize);
  }
  return 0;
}

int difftest_state() {
  for (int i = 0; i < NUM_CORES; i++) {
    if (difftest[i]->get_trap_valid()) {
      return difftest[i]->get_trap_code();
    }
  }
  return -1;
}

void printInstrCommit(const instr_commit_t& instr) {
  printf("valid: %d, pc: %lx, inst: %x, skip: %d, isRVC: %d, fused: %d, rfwen: %d, fpwen: %d, pdst0: %u, dst0: %d, lqidx: %d, sqidx: %d, robidx: %u, isLoad: %d, isStore: %d, lsrc0: %u, lsrc1: %u, lsrc2: %u, psrc0: %u, psrc1: %u, psrc2: %u, cycleCnt: %lu\n",
         instr.valid, instr.pc, instr.inst, instr.skip, instr.isRVC, instr.fused, instr.rfwen, instr.fpwen, instr.wpdest, instr.wdest, instr.lqidx, instr.sqidx, instr.robidx, instr.isLoad, instr.isStore, instr.lsrc0, instr.lsrc1, instr.lsrc2, instr.psrc0, instr.psrc1, instr.psrc2, instr.cycleCnt);
}

void make_ldst_events(int core_id){
  for (int i = 0; i < DIFFTEST_COMMIT_WIDTH; i++) {
    auto df = difftest[core_id];
    load_event_t *mem_event = df->get_load_event(i);

    if (mem_event->valid == 0) {
      continue;
    }

    // noarmal load
    // Note: bit(1, 0) are size, DO NOT CHANGE
    // bit encoding: | load 0 | is unsigned(1bit) | size(2bit) |
    // def lb       = "b0000".U
    // def lh       = "b0001".U
    // def lw       = "b0010".U
    // def ld       = "b0011".U
    // def lbu      = "b0100".U
    // def lhu      = "b0101".U
    // def lwu      = "b0110".U

    // normal store
    // bit encoding: | store 00 | size(2bit) |
    // def sb       = "b0000".U
    // def sh       = "b0001".U
    // def sw       = "b0010".U
    // def sd       = "b0011".U

    // analyze dependency
    const instr_commit_t *instr = df->get_instr_commit(i);
    uint64_t dependency_addr = 0;
    uint64_t dependency_x = 0;
    dependency_x |= instr->robidx & ROB_IDX_MUSK;
    dependency_addr |= (uint64_t)(instr->psrc0 & REG_MUSK) << (uint64_t)RegOffset::Psrc0;
    dependency_addr |= (uint64_t)(instr->psrc1 & REG_MUSK) << (uint64_t)RegOffset::Psrc1;
    dependency_addr |= (uint64_t)(instr->psrc2 & REG_MUSK) << (uint64_t)RegOffset::Psrc2;
    dependency_addr |= (uint64_t)(instr->lsrc0 & REG_MUSK) << (uint64_t)RegOffset::Lsrc0;
    dependency_addr |= (uint64_t)(instr->lsrc1 & REG_MUSK) << (uint64_t)RegOffset::Lsrc1;
    dependency_addr |= (uint64_t)(instr->lsrc2 & REG_MUSK) << (uint64_t)RegOffset::Lsrc2;
    dependency_addr |= (uint64_t)(instr->wdest & REG_MUSK) << (uint64_t)RegOffset::Ldst0;
    dependency_addr |= (uint64_t)(instr->wpdest & REG_MUSK) << (uint64_t)RegOffset::Pdst0;
    dependency_x |= (uint64_t)instr->inst << 32;

    // printInstrCommit(*instr);

    // load or Store
    if (mem_event->fuType == 0xC || mem_event->fuType == 0xD) {
      dependency_x |= mem_event->x;

      int len = 0;
      switch (mem_event->opType) {
        case 0: len = 1; break;
        case 1: len = 2; break;
        case 2: len = 4; break;
        case 3: len = 8; break;
        case 4: len = 1; break;
        case 5: len = 2; break;
        case 6: len = 4; break;
        default:
          printf("Unknown fuOpType: 0x%x\n", mem_event->opType);
      }

      uint64_t data = df->get_commit_data(i);
      // switch (opType) {
      //   case 0: data = (int64_t)(int8_t)data; break;
      //   case 1: data = (int64_t)(int16_t)data; break;
      //   case 2: data = (int64_t)(int32_t)data; break;
      // }

      // store with wrong optype
      if (mem_event->fuType == 0xD && mem_event->opType > 3) {
        printf("ERROR: Store with wrong optype detected!\n");
      }

      EventType ty = EventType::Invalid;

      // load
      if (mem_event->fuType == 0xC) {
        dependency_x |= 1ULL << (uint64_t)XOffset::IsLoad;
        ty = EventType::LoadCommit;
      }

      // store
      if (mem_event->fuType == 0xD) {
        dependency_x |= 1ULL << (uint64_t)XOffset::IsStore;
        ty = EventType::StoreCommit;
        auto sqidx = df->get_instr_commit(i)->sqidx;
        data = df->get_x_event(sqidx)->data;
      }
      for (int j = 0; j < len; j++) {
        uint8_t data_byte = (data >> (j * 8)) & 0xFF;
        Event event(ty, core_id, mem_event->paddr + j, data_byte, mem_event->x, mem_event->cycleCnt);
        mcm_event_push(event);
      }
    }
    
    Event event(EventType::Dependency, core_id, dependency_addr, 0, dependency_x, instr->cycleCnt);
    mcm_event_push(event);
  }
}

void make_atomic_events(int coreid) {
  auto df = difftest[coreid];
  auto atomic_event = df->get_atomic_event();
  if (atomic_event->resp) {
    uint64_t ATOMIC_FLAG = 1ULL << (uint64_t)XOffset::IsAtomic;

    uint64_t addr = atomic_event->addr & 0xfffffffffffffff8;
    uint8_t mask = atomic_event->mask;
    uint64_t data = atomic_event->data;
    uint8_t fuop = atomic_event->fuop;
    uint64_t out = atomic_event->out;
    uint64_t cycleCnt = atomic_event->cycleCnt;
    uint64_t x = atomic_event->x | ATOMIC_FLAG;
    // printf("atomic event: addr: %016lx, mask: %x, data: %016lx, fuop: %x, out: %016lx, time: %ld  \n", addr, mask, data, fuop, out, cycleCnt);

    // atomics
    // bit(1, 0) are size
    // since atomics use a different fu type
    // so we can safely reuse other load/store's encodings
    // bit encoding: | optype(4bit) | size (2bit) |
    // def lr_w      = "b000010".U  // 0x02
    // def sc_w      = "b000110".U  // 0x06
    // def amoswap_w = "b001010".U  // 0x0A
    // def amoadd_w  = "b001110".U  // 0x0E
    // def amoxor_w  = "b010010".U  // 0x12
    // def amoand_w  = "b010110".U  // 0x16
    // def amoor_w   = "b011010".U  // 0x1A
    // def amomin_w  = "b011110".U  // 0x1E
    // def amomax_w  = "b100010".U  // 0x22
    // def amominu_w = "b100110".U  // 0x26
    // def amomaxu_w = "b101010".U  // 0x2A
    // def lr_d      = "b000011".U  // 0x03
    // def sc_d      = "b000111".U  // 0x07
    // def amoswap_d = "b001011".U  // 0x0B
    // def amoadd_d  = "b001111".U  // 0x0F
    // def amoxor_d  = "b010011".U  // 0x13
    // def amoand_d  = "b010111".U  // 0x17
    // def amoor_d   = "b011011".U  // 0x1B
    // def amomin_d  = "b011111".U  // 0x1F
    // def amomax_d  = "b100011".U  // 0x23
    // def amominu_d = "b100111".U  // 0x27
    // def amomaxu_d = "b101011".U  // 0x2B

    if (!(mask == 0xf || mask == 0xf0 || mask == 0xff)) {
      printf("atomic mask error: %x\n", mask);
      exit(-1);
    }

    EventType ty = EventType::StoreGlobal;
    uint64_t ret;
    

    if (mask == 0xff) {
      switch (fuop) {
        case 0x02: case 0x03: ret = out; ty = EventType::LoadCommit; break;
        // if sc fails(aka atomicOut == 1), no update to goldenmem
        case 0x06: case 0x07: if (out == 1) return; ret = data; break;
        case 0x0A: case 0x0B: ret = data; break;
        case 0x0E: case 0x0F: ret = out + data; break;
        case 0x12: case 0x13: ret = out ^ data; break;
        case 0x16: case 0x17: ret = out & data; break;
        case 0x1A: case 0x1B: ret = out | data; break;
        case 0x1E: case 0x1F: ret = ((int64_t)out < (int64_t)data) ? out : data; break;
        case 0x22: case 0x23: ret = ((int64_t)out > (int64_t)data) ? out : data; break;
        case 0x26: case 0x27: ret = (out < data) ? out : data; break;
        case 0x2A: case 0x2B: ret = (out > data) ? out : data; break;
    
        default: printf("Unknown atomic fuOpType: 0x%x\n", fuop);
      }
    }

    if (mask == 0xf || mask == 0xf0) {
      uint32_t rs = (uint32_t)data;  // rs2
      uint32_t t  = (uint32_t)out;   // original value

      switch (fuop) {
        case 0x02: case 0x03: ret = t; ty = EventType::LoadCommit; break;
        // if sc fails(aka atomicOut == 1), no update to goldenmem
        case 0x06: case 0x07: if (t == 1) return; ret = rs; break;
        case 0x0A: case 0x0B: ret = rs; break;
        case 0x0E: case 0x0F: ret = t + rs; break;
        case 0x12: case 0x13: ret = t ^ rs; break;
        case 0x16: case 0x17: ret = t & rs; break;
        case 0x1A: case 0x1B: ret = t | rs; break;
        case 0x1E: case 0x1F: ret = ((int32_t)t < (int32_t)rs) ? t : rs; break;
        case 0x22: case 0x23: ret = ((int32_t)t > (int32_t)rs) ? t : rs; break;
        case 0x26: case 0x27: ret = (t < rs) ? t : rs; break;
        case 0x2A: case 0x2B: ret = (t > rs) ? t : rs; break;
        default: printf("Unknown atomic fuOpType: 0x%x\n", fuop);
      }
      if (mask == 0xf0) {
        ret = (ret << 32);
        out = (out << 32);
      }
    }

    for (int i = 0; i < 8; i++) {
      // only keep aqrl bits in the first event, order check just need one time for an amo
      if (i > 0) {
        x &= ~((1ULL << (uint64_t)XOffset::IsAQ) | (1ULL << (uint64_t)XOffset::IsRL));
      }
      
      if (mask & (1 << i)) {
        
        if (ty == EventType::StoreGlobal) {
          
          // sc is not a read-modify-write instruction so we don't need to push load events.
          if (fuop != 6 && fuop != 7) {
            Event event = Event(EventType::LoadLocal, coreid, addr + i, (out >> (i * 8)) & 0xFF, x, cycleCnt);
            mcm_event_push(event);
            event.ty = EventType::LoadCommit;
            mcm_event_push(event);
          }

          Event event = Event(EventType::StoreCommit, coreid, addr + i, (ret >> (i * 8)) & 0xFF, x, cycleCnt);
          mcm_event_push(event);
          event.ty = EventType::StoreLocal;
          mcm_event_push(event);
        } else {
          Event event(EventType::LoadLocal, coreid, addr + i, (ret >> (i * 8)) & 0xFF, x, cycleCnt);
          mcm_event_push(event);
        }

        Event event(ty, coreid, addr + i, (ret >> (i * 8)) & 0xFF, x, cycleCnt);
        mcm_event_push(event);

      }
    }
  }
}

void make_fence_events(int coreid) {
  auto df = difftest[coreid];
  for (int i = 0; i < DIFFTEST_COMMIT_WIDTH; i++) {
    load_event_t *mem_event = df->get_load_event(i);

    if (mem_event->valid == 0) {
      continue;
    }

    // reuse fence event to synchronize robidx with mcm checker
    uint64_t IsBidIdx_Flag = 1ULL << (uint64_t)XOffset::IsRobIdx;
    Event event(EventType::Fence, coreid, 0, 0, mem_event->x | IsBidIdx_Flag, mem_event->cycleCnt);
    mcm_event_push(event);

    // def fence        = "b0011".U // 0x3
    if (mem_event->fuType == 0x3) {
      Event event(EventType::Fence, coreid, 0, 0, mem_event->x, mem_event->cycleCnt);
      mcm_event_push(event);
    }

  }
}

void make_mcm_events(int coreid){
  make_ldst_events(coreid);
  make_atomic_events(coreid);
  make_fence_events(coreid);
}

int difftest_step() {
  for (int i = 0; i < NUM_CORES; i++) {
    make_mcm_events(i);
    mcm_check();
    int ret = difftest[i]->step();
    if (ret) {
      return ret;
    }
  }
  return 0;
}


Difftest::Difftest(int coreid) : id(coreid) {
  state = new DiffState();
  clear_step();
}

void Difftest::update_nemuproxy(int coreid, size_t ram_size = 0) {
  proxy = new DIFF_PROXY(coreid, ram_size);
}

int Difftest::step() {
  progress = false;
  ticks++;

#ifdef BASIC_DIFFTEST_ONLY
  proxy->regcpy(ref_regs_ptr, REF_TO_DUT);
  dut.csr.this_pc = ref.csr.this_pc;
#else
  // TODO: update nemu/xs to fix this_pc comparison
  dut.csr.this_pc = dut.commit[0].pc;
#endif

  if (check_timeout()) {
    return 1;
  }
  do_first_instr_commit();
  if (do_store_check()) {
    return 1;
  }

#ifdef DEBUG_GOLDENMEM
  if (do_golden_memory_update()) {
    return 1;
  }
#endif

  if (!has_commit) {
    return 0;
  }

#ifdef DEBUG_REFILL
  if (do_irefill_check() || do_drefill_check() || do_ptwrefill_check() ) {
    return 1;
  }
#endif

#ifdef DEBUG_L2TLB
  if (do_l2tlb_check()) {
    return 1;
  }
#endif

#ifdef DEBUG_L1TLB
  if (do_itlb_check() || do_ldtlb_check() || do_sttlb_check()) {
    return 1;
  }
#endif

#ifdef DEBUG_MODE_DIFF
  // skip load & store insts in debug mode
  // for other insts copy inst content to ref's dummy debug module
  for(int i = 0; i < DIFFTEST_COMMIT_WIDTH; i++){
    if(DEBUG_MEM_REGION(dut.commit[i].valid, dut.commit[i].pc))
      debug_mode_copy(dut.commit[i].pc, dut.commit[i].isRVC ? 2 : 4, dut.commit[i].inst);
  }

#endif

  num_commit = 0; // reset num_commit this cycle to 0
  // interrupt has the highest priority
  if (dut.event.interrupt) {
    dut.csr.this_pc = dut.event.exceptionPC;
    do_interrupt();
  } else if (dut.event.exception) {
    // We ignored instrAddrMisaligned exception (0) for better debug interface
    // XiangShan should always support RVC, so instrAddrMisaligned will never happen
    // TODO: update NEMU, for now, NEMU will update pc when exception happen
    dut.csr.this_pc = dut.event.exceptionPC;
    do_exception();
  } else {
    // TODO: is this else necessary?
    for (int i = 0; i < DIFFTEST_COMMIT_WIDTH && dut.commit[i].valid; i++) {
      do_instr_commit(i);
      dut.commit[i].valid = 0;
      num_commit++;
      // TODO: let do_instr_commit return number of instructions in this uop
      if (dut.commit[i].fused) {
        num_commit++;
      }
    }
  }

  if (!progress) {
    return 0;
  }

  proxy->regcpy(ref_regs_ptr, REF_TO_DUT);

  if (num_commit > 0) {
    state->record_group(dut.commit[0].pc, num_commit);
  }

  // swap nemu_pc and ref.csr.this_pc for comparison
  uint64_t nemu_next_pc = ref.csr.this_pc;
  ref.csr.this_pc = nemu_this_pc;
  nemu_this_pc = nemu_next_pc;

  // FIXME: the following code is dirty
  if (dut_regs_ptr[72] != ref_regs_ptr[72]) {  // Ignore difftest for MIP
    ref_regs_ptr[72] = dut_regs_ptr[72];
  }

  if (memcmp(dut_regs_ptr, ref_regs_ptr, DIFFTEST_NR_REG * sizeof(uint64_t))) {
    display();
    for (int i = 0; i < DIFFTEST_NR_REG; i ++) {
      if (dut_regs_ptr[i] != ref_regs_ptr[i]) {
        printf("%7s different at pc = 0x%010lx, right= 0x%016lx, wrong = 0x%016lx\n",
            reg_name[i], ref.csr.this_pc, ref_regs_ptr[i], dut_regs_ptr[i]);
      }
    }
    return 1;
  }

  return 0;
}

void Difftest::do_interrupt() {
  state->record_abnormal_inst(dut.event.exceptionPC, dut.event.exceptionInst, RET_INT, dut.event.interrupt);
  proxy->raise_intr(dut.event.interrupt | (1ULL << 63));
  progress = true;
}

void Difftest::do_exception() {
  state->record_abnormal_inst(dut.event.exceptionPC, dut.event.exceptionInst, RET_EXC, dut.event.exception);
  if (dut.event.exception == 12 || dut.event.exception == 13 || dut.event.exception == 15) {
    // printf("exception cause: %d\n", dut.event.exception);
    struct ExecutionGuide guide;
    guide.force_raise_exception = true;
    guide.exception_num = dut.event.exception;
    guide.mtval = dut.csr.mtval;
    guide.stval = dut.csr.stval;
    guide.force_set_jump_target = false;
    proxy->guided_exec(&guide);
  } else {
  #ifdef DEBUG_MODE_DIFF
    if(DEBUG_MEM_REGION(true, dut.event.exceptionPC)){
      // printf("exception instr is %x\n", dut.event.exceptionInst);
      debug_mode_copy(dut.event.exceptionPC, 4, dut.event.exceptionInst);
    }
  #endif
    proxy->exec(1);
  }
  progress = true;
}

void Difftest::do_instr_commit(int i) {
  progress = true;
  update_last_commit();

  // store the writeback info to debug array
#ifdef BASIC_DIFFTEST_ONLY
  uint64_t commit_pc = ref.csr.this_pc;
  uint64_t commit_instr = 0x0;
#else
  uint64_t commit_pc = dut.commit[i].pc;
  uint64_t commit_instr = dut.commit[i].inst;
#endif
  state->record_inst(commit_pc, commit_instr, (dut.commit[i].rfwen | dut.commit[i].fpwen), dut.commit[i].wdest, get_commit_data(i), dut.commit[i].lqidx, dut.commit[i].sqidx, dut.commit[i].robidx, dut.commit[i].isLoad, dut.commit[i].isStore, dut.commit[i].skip != 0);

#ifdef DEBUG_MODE_DIFF
  int spike_invalid = test_spike();
  if (!spike_invalid && (IS_DEBUGCSR(commit_instr) || IS_TRIGGERCSR(commit_instr))) {
    char inst_str[32];
    char dasm_result[64] = {0};
    sprintf(inst_str, "%08x", commit_instr);
    spike_dasm(dasm_result, inst_str);
    printf("s0 is %016lx ", dut.regs.gpr[8]);
    printf("pc is %lx %s\n", commit_pc, dasm_result);
  }
#endif

  // sync lr/sc reg status
  if (dut.lrsc.valid) {
    struct SyncState sync;
    sync.lrscValid = dut.lrsc.success;
    proxy->uarchstatus_cpy((uint64_t*)&sync, DUT_TO_REF); // sync lr/sc microarchitectural regs
    // clear SC instruction valid bit
    dut.lrsc.valid = 0;
  }

  bool realWen = (dut.commit[i].rfwen && dut.commit[i].wdest != 0) || (dut.commit[i].fpwen);

  // MMIO accessing should not be a branch or jump, just +2/+4 to get the next pc
  // to skip the checking of an instruction, just copy the reg state to reference design
  if (dut.commit[i].skip || (DEBUG_MODE_SKIP(dut.commit[i].valid, dut.commit[i].pc, dut.commit[i].inst))) {
    proxy->regcpy(ref_regs_ptr, REF_TO_DIFFTEST);
    ref.csr.this_pc += dut.commit[i].isRVC ? 2 : 4;
    if (realWen) {
      // We use the physical register file to get wdata
      // TODO: what if skip with fpwen?
      ref_regs_ptr[dut.commit[i].wdest] = get_commit_data(i);
      // printf("Debug Mode? %x is ls? %x\n", DEBUG_MEM_REGION(dut.commit[i].valid, dut.commit[i].pc), IS_LOAD_STORE(dut.commit[i].inst));
      // printf("skip %x %x %x %x %x\n", dut.commit[i].pc, dut.commit[i].inst, get_commit_data(i), dut.commit[i].wpdest, dut.commit[i].wdest);
    }
    proxy->regcpy(ref_regs_ptr, DIFFTEST_TO_REF);
    return;
  }

  // single step exec
  proxy->exec(1);
  // when there's a fused instruction, let proxy execute one more instruction.
  if (dut.commit[i].fused) {
    proxy->exec(1);
  }

  // Handle load instruction carefully for SMP
  if (NUM_CORES > 1) {
    if (dut.load[i].fuType == 0xC || dut.load[i].fuType == 0xF) {
      proxy->regcpy(ref_regs_ptr, REF_TO_DUT);
      if (realWen && ref_regs_ptr[dut.commit[i].fpwen * 32 + dut.commit[i].wdest] != get_commit_data(i)) {
        // printf("---[DIFF Core%d] This load instruction gets rectified!\n", this->id);
        // printf("---    ltype: 0x%x paddr: 0x%lx wen: 0x%x wdst: 0x%x wdata: 0x%lx pc: 0x%lx\n", dut.load[i].opType, dut.load[i].paddr, dut.commit[i].wen, dut.commit[i].wdest, get_commit_data(i), dut.commit[i].pc);
        uint64_t golden;
        int len = 0;
        if (dut.load[i].fuType == 0xC) {
          switch (dut.load[i].opType) {
            case 0: len = 1; break;
            case 1: len = 2; break;
            case 2: len = 4; break;
            case 3: len = 8; break;
            case 4: len = 1; break;
            case 5: len = 2; break;
            case 6: len = 4; break;
            default:
              printf("Unknown fuOpType: 0x%x\n", dut.load[i].opType);
          }
        } else {  // dut.load[i].fuType == 0xF
          if (dut.load[i].opType % 2 == 0) {
            len = 4;
          } else {  // dut.load[i].opType % 2 == 1
            len = 8;
          }
        }
        read_goldenmem(dut.load[i].paddr, &golden, len);
        if (dut.load[i].fuType == 0xC) {
          switch (dut.load[i].opType) {
            case 0: golden = (int64_t)(int8_t)golden; break;
            case 1: golden = (int64_t)(int16_t)golden; break;
            case 2: golden = (int64_t)(int32_t)golden; break;
          }
        }
        // printf("---    golden: 0x%lx  original: 0x%lx\n", golden, ref_regs_ptr[dut.commit[i].wdest]);
        if (golden == get_commit_data(i)) {
          proxy->memcpy(dut.load[i].paddr, &golden, len, DUT_TO_DIFFTEST);
          if (realWen) {
            ref_regs_ptr[dut.commit[i].fpwen * 32 + dut.commit[i].wdest] = get_commit_data(i);
            proxy->regcpy(ref_regs_ptr, DUT_TO_DIFFTEST);
          }
        } else if (dut.load[i].fuType == 0xF) {  //  atomic instr carefully handled
          proxy->memcpy(dut.load[i].paddr, &golden, len, DIFFTEST_TO_REF);
          if (realWen) {
            ref_regs_ptr[dut.commit[i].fpwen * 32 + dut.commit[i].wdest] = get_commit_data(i);
            proxy->regcpy(ref_regs_ptr, DUT_TO_DIFFTEST);
          }
        } else {
#ifdef DEBUG_SMP
          // goldenmem check failed as well, raise 
          printf("---  SMP difftest mismatch!\n");
          printf("---  Trying to probe local data of another core\n");
          uint64_t buf;
          difftest[(NUM_CORES-1) - this->id]->proxy->memcpy(dut.load[i].paddr, &buf, len, DIFFTEST_TO_DUT);
          printf("---    content: %lx\n", buf);
#else
          proxy->memcpy(dut.load[i].paddr, &golden, len, DUT_TO_DIFFTEST);
          if (realWen) {
            ref_regs_ptr[dut.commit[i].fpwen * 32 + dut.commit[i].wdest] = get_commit_data(i);
            proxy->regcpy(ref_regs_ptr, DUT_TO_DIFFTEST);
          }
#endif
        }
      }
    }
  }
}

void Difftest::do_first_instr_commit() {
  if (!has_commit && dut.commit[0].valid) {
#ifndef BASIC_DIFFTEST_ONLY
    if (dut.commit[0].pc != FIRST_INST_ADDRESS) {
      return;
    }
#endif
    printf("The first instruction of core %d has commited. Difftest enabled. \n", id);
    has_commit = 1;
    nemu_this_pc = FIRST_INST_ADDRESS;

    proxy->load_flash_bin(get_flash_path(), get_flash_size());
    proxy->memcpy(PMEM_BASE, get_img_start(), get_img_size(), DIFFTEST_TO_REF);
    // Use a temp variable to store the current pc of dut
    uint64_t dut_this_pc = dut.csr.this_pc;
    // NEMU should always start at FIRST_INST_ADDRESS
    dut.csr.this_pc = FIRST_INST_ADDRESS;
    proxy->regcpy(dut_regs_ptr, DIFFTEST_TO_REF);
    dut.csr.this_pc = dut_this_pc;
    // Do not reconfig simulator 'proxy->update_config(&nemu_config)' here:
    // If this is main sim thread, simulator has its own initial config
    // If this process is checkpoint wakeuped, simulator's config has already been updated,
    // do not override it.
  }
}

int Difftest::do_store_check() {
  for (int i = 0; i < DIFFTEST_STORE_WIDTH; i++) {
    if (!dut.store[i].valid) {
      return 0;
    }
    auto addr = dut.store[i].addr;
    auto data = dut.store[i].data;
    auto mask = dut.store[i].mask;
    if (proxy->store_commit(&addr, &data, &mask)) {
      display();
      printf("Mismatch for store commits %d: \n", i);
      printf("  REF commits addr 0x%lx, data 0x%lx, mask 0x%x\n", addr, data, mask);
      printf("  DUT commits addr 0x%lx, data 0x%lx, mask 0x%x\n",
        dut.store[i].addr, dut.store[i].data, dut.store[i].mask);
      return 1;
    }
    dut.store[i].valid = 0;
  }
  return 0;
}

int Difftest::do_refill_check(int cacheid) {
  static uint64_t last_valid_addr = 0;
  char buf[512];
  refill_event_t dut_refill = cacheid == PAGECACHEID ? dut.ptw_refill : cacheid == DCACHEID ? dut.d_refill : dut.i_refill ;
  const char* name = cacheid == PAGECACHEID ? "PageCache" : cacheid == DCACHEID ? "DCache" : "ICache";
  dut_refill.addr = dut_refill.addr - dut_refill.addr % 64;
  if (dut_refill.valid == 1 && dut_refill.addr != last_valid_addr) {
    last_valid_addr = dut_refill.addr;
    if(!in_pmem(dut_refill.addr)){
      // speculated illegal mem access should be ignored
      return 0;
    }
    for (int i = 0; i < 8; i++) {
      read_goldenmem(dut_refill.addr + i*8, &buf, 8);
      // printf("Refill: addr: 0x%016lx, data: 0x%016lx\n",dut_refill.addr + i*8, dut_refill.data[i]);
      if (dut_refill.data[i] != *((uint64_t*)buf)) {
        printf("%s Refill test failed!\n",name);
        printf("addr: %lx\nGold: ", dut_refill.addr);
        for (int j = 0; j < 8; j++) {
          read_goldenmem(dut_refill.addr + j*8, &buf, 8);
          printf("%016lx", *((uint64_t*)buf));
        }
        printf("\nCore: ");
        for (int j = 0; j < 8; j++) {
          printf("%016lx", dut_refill.data[j]);
        }
        printf("\n");
        return 1;
      }
    }
  }
  return 0;
}

int Difftest::do_irefill_check() {
    return do_refill_check(ICACHEID);
}

int Difftest::do_drefill_check() {
    return do_refill_check(DCACHEID);
}

int Difftest::do_ptwrefill_check() {
    return do_refill_check(PAGECACHEID);
}

int Difftest::do_l1tlb_check(int l1tlbid) {

  PTE pte;
  uint64_t paddr;
  uint8_t difftest_level;

  if (l1tlbid == STTLBID) {
    for (int i = 0; i < DIFFTEST_STTLB_WIDTH; i++) {
      if (!dut.sttlb[i].valid) {
        continue;
      }

      uint64_t pg_base = dut.sttlb[i].satp << 12;
      for (difftest_level = 0; difftest_level < 3; difftest_level++) {
        paddr = pg_base + VPNi(dut.sttlb[i].vpn, difftest_level) * sizeof(uint64_t);
        read_goldenmem(paddr, &pte.val, 8);
        if (!pte.v || pte.r || pte.x || pte.w || difftest_level == 2) {
          break;
        }
        pg_base = pte.ppn << 12;
      }

      dut.sttlb[i].ppn = dut.sttlb[i].ppn >> (2 - difftest_level) * 9 << (2 - difftest_level) * 9;
      if (pte.difftest_ppn != dut.sttlb[i].ppn ) {
        printf("STTLB resp test of core %d index %d failed! vpn = %lx\n", id, i, dut.sttlb[i].vpn);
        printf("  REF commits ppn 0x%lx, DUT commits ppn 0x%lx\n", pte.difftest_ppn, dut.sttlb[i].ppn);
        return 1;
      }
    }
    return 0;
  }
  if (l1tlbid == LDTLBID) {
    for (int i = 0; i < DIFFTEST_LDTLB_WIDTH; i++) {
      if (!dut.ldtlb[i].valid) {
        continue;
      }

      uint64_t pg_base = dut.ldtlb[i].satp << 12;
      for (difftest_level = 0; difftest_level < 3; difftest_level++) {
        paddr = pg_base + VPNi(dut.ldtlb[i].vpn, difftest_level) * sizeof(uint64_t);
        read_goldenmem(paddr, &pte.val, 8);
        if (!pte.v || pte.r || pte.x || pte.w || difftest_level == 2) {
          break;
        }
        pg_base = pte.ppn << 12;
      }

      dut.ldtlb[i].ppn = dut.ldtlb[i].ppn >> (2 - difftest_level) * 9 << (2 - difftest_level) * 9;
      if (pte.difftest_ppn != dut.ldtlb[i].ppn ) {
        printf("LDTLB resp test of core %d index %d failed! vpn = %lx\n", id, i, dut.ldtlb[i].vpn);
        printf("  REF commits ppn 0x%lx, DUT commits ppn 0x%lx\n", pte.difftest_ppn, dut.ldtlb[i].ppn);
        return 1;
      }
    }
    return 0;
  }
  if (l1tlbid == ITLBID) {
    for (int i = 0; i < DIFFTEST_ITLB_WIDTH; i++) {
      if (!dut.itlb[i].valid) {
        continue;
      }
      uint64_t pg_base = dut.itlb[i].satp << 12;
      for (difftest_level = 0; difftest_level < 3; difftest_level++) {
        paddr = pg_base + VPNi(dut.itlb[i].vpn, difftest_level) * sizeof(uint64_t);
        read_goldenmem(paddr, &pte.val, 8);
        if (!pte.v || pte.r || pte.x || pte.w || difftest_level == 2) {
          break;
        }
        pg_base = pte.ppn << 12;
      }

      dut.itlb[i].ppn = dut.itlb[i].ppn >> (2 - difftest_level) * 9 << (2 - difftest_level) * 9;
      if (pte.difftest_ppn != dut.itlb[i].ppn) {
        printf("ITLB resp test of core %d index %d failed! vpn = %lx\n", id, i, dut.itlb[i].vpn);
        printf("  REF commits ppn 0x%lx, DUT commits ppn 0x%lx\n", pte.difftest_ppn, dut.itlb[i].ppn);
        return 1;
      }
    }
    return 0;
  }
}

int Difftest::do_itlb_check() {
    return do_l1tlb_check(ITLBID);
}

int Difftest::do_ldtlb_check() {
    return do_l1tlb_check(LDTLBID);
}

int Difftest::do_sttlb_check() {
    return do_l1tlb_check(STTLBID);
}

int Difftest::do_l2tlb_check() {
  for (int i = 0; i < DIFFTEST_PTW_WIDTH; i++) {
    if (!dut.l2tlb[i].valid) {
      continue;
    }

    PTE pte;
    uint64_t pg_base = dut.l2tlb[i].satp << 12;
    uint64_t paddr;
    uint8_t difftest_level;

    for (difftest_level = 0; difftest_level < 3; difftest_level++) {
      paddr = pg_base + VPNi(dut.l2tlb[i].vpn, difftest_level) * sizeof(uint64_t);
      read_goldenmem(paddr, &pte.val, 8);
      if (!pte.v || pte.r || pte.x || pte.w || difftest_level == 2) {
        break;
      }
      pg_base = pte.ppn << 12;
    }

    bool difftest_pf = !pte.v || (!pte.r && pte.w);
    if (pte.difftest_ppn != dut.l2tlb[i].ppn || pte.difftest_perm != dut.l2tlb[i].perm || difftest_level != dut.l2tlb[i].level || difftest_pf != dut.l2tlb[i].pf) {
      printf("L2TLB resp test of core %d index %d failed! vpn = %lx\n", id, i, dut.l2tlb[i].vpn);
      printf("  REF commits ppn 0x%lx, perm 0x%02x, level %d, pf %d\n", pte.difftest_ppn, pte.difftest_perm, difftest_level, !pte.difftest_v);
      printf("  DUT commits ppn 0x%lx, perm 0x%02x, level %d, pf %d\n", dut.l2tlb[i].ppn, dut.l2tlb[i].perm, dut.l2tlb[i].level, dut.l2tlb[i].pf);
      return 1;
    }
  }
  return 0;
}

inline int handle_atomic(int coreid, uint64_t atomicAddr, uint64_t atomicData, uint64_t atomicMask, uint8_t atomicFuop, uint64_t atomicOut) {
  // We need to do atmoic operations here so as to update goldenMem
  if (!(atomicMask == 0xf || atomicMask == 0xf0 || atomicMask == 0xff)) {
    printf("Unrecognized mask: %lx\n", atomicMask);
    return 1;
  }

  if (atomicMask == 0xff) {
    uint64_t rs = atomicData;  // rs2
    uint64_t t  = atomicOut;   // original value
    uint64_t ret;
    uint64_t mem;
    read_goldenmem(atomicAddr, &mem, 8);
    if (mem != t && atomicFuop != 007 && atomicFuop != 003) {  // ignore sc_d & lr_d
      printf("Core %d atomic instr mismatch goldenMem, mem: 0x%lx, t: 0x%lx, op: 0x%x, addr: 0x%lx\n", coreid, mem, t, atomicFuop, atomicAddr);
      return 1;
    }
    switch (atomicFuop) {
      case 002: case 003: ret = t; break;
      // if sc fails(aka atomicOut == 1), no update to goldenmem
      case 006: case 007: if (t == 1) return 0; ret = rs; break;
      case 012: case 013: ret = rs; break;
      case 016: case 017: ret = t+rs; break;
      case 022: case 023: ret = (t^rs); break;
      case 026: case 027: ret = t & rs; break;
      case 032: case 033: ret = t | rs; break;
      case 036: case 037: ret = ((int64_t)t < (int64_t)rs)? t : rs; break;
      case 042: case 043: ret = ((int64_t)t > (int64_t)rs)? t : rs; break;
      case 046: case 047: ret = (t < rs) ? t : rs; break;
      case 052: case 053: ret = (t > rs) ? t : rs; break;
      default: printf("Unknown atomic fuOpType: 0x%x\n", atomicFuop);
    }
    update_goldenmem(atomicAddr, &ret, atomicMask, 8);
  }

  if (atomicMask == 0xf || atomicMask == 0xf0) {
    uint32_t rs = (uint32_t)atomicData;  // rs2
    uint32_t t  = (uint32_t)atomicOut;   // original value
    uint32_t ret;
    uint32_t mem;
    uint64_t mem_raw;
    uint64_t ret_sel;
    atomicAddr = (atomicAddr & 0xfffffffffffffff8);
    read_goldenmem(atomicAddr, &mem_raw, 8);

    if (atomicMask == 0xf)
      mem = (uint32_t)mem_raw;
    else
      mem = (uint32_t)(mem_raw >> 32);

    if (mem != t && atomicFuop != 006 && atomicFuop != 002) {  // ignore sc_w & lr_w
      printf("Core %d atomic instr mismatch goldenMem, rawmem: 0x%lx mem: 0x%x, t: 0x%x, op: 0x%x, addr: 0x%lx\n", coreid, mem_raw, mem, t, atomicFuop, atomicAddr);
      return 1;
    }
    switch (atomicFuop) {
      case 002: case 003: ret = t; break;
      // if sc fails(aka atomicOut == 1), no update to goldenmem
      case 006: case 007: if (t == 1) return 0; ret = rs; break;
      case 012: case 013: ret = rs; break;
      case 016: case 017: ret = t+rs; break;
      case 022: case 023: ret = (t^rs); break;
      case 026: case 027: ret = t & rs; break;
      case 032: case 033: ret = t | rs; break;
      case 036: case 037: ret = ((int32_t)t < (int32_t)rs)? t : rs; break;
      case 042: case 043: ret = ((int32_t)t > (int32_t)rs)? t : rs; break;
      case 046: case 047: ret = (t < rs) ? t : rs; break;
      case 052: case 053: ret = (t > rs) ? t : rs; break;
      default: printf("Unknown atomic fuOpType: 0x%x\n", atomicFuop);
    }
    ret_sel = ret;
    if (atomicMask == 0xf0)
      ret_sel = (ret_sel << 32);
    update_goldenmem(atomicAddr, &ret_sel, atomicMask, 8);
  }
  return 0;
}

void dumpGoldenMem(const char* banner, uint64_t addr, uint64_t time) {
#ifdef DEBUG_REFILL
  char buf[512];
  if (addr == 0) {
    return;
  }
  printf("============== %s =============== time = %ld\ndata: ", banner, time);
    for (int i = 0; i < 8; i++) {
      read_goldenmem(addr + i*8, &buf, 8);
      printf("%016lx", *((uint64_t*)buf));
    }
    printf("\n");
#endif
}

#ifdef DEBUG_GOLDENMEM
int Difftest::do_golden_memory_update() {
  // Update Golden Memory info

  if (ticks == 100) {
    dumpGoldenMem("Init", track_instr, ticks);
  }

  for(int i = 0; i < DIFFTEST_SBUFFER_RESP_WIDTH; i++){
    if (dut.sbuffer[i].resp) {
      dut.sbuffer[i].resp = 0;
      update_goldenmem(dut.sbuffer[i].addr, dut.sbuffer[i].data, dut.sbuffer[i].mask, 64);
      if (dut.sbuffer[i].addr == track_instr) {
        dumpGoldenMem("Store", track_instr, ticks);
      }
    }
  }

  if (dut.atomic.resp) {
    dut.atomic.resp = 0;
    int ret = handle_atomic(id, dut.atomic.addr, dut.atomic.data, dut.atomic.mask, dut.atomic.fuop, dut.atomic.out);
    if (dut.atomic.addr == track_instr) {
      dumpGoldenMem("Atmoic", track_instr, ticks);
    }
    if (ret) return ret;
  }
  return 0;
}
#endif

int Difftest::check_timeout() {
  // check whether there're any commits since the simulation starts
  if (!has_commit && ticks > last_commit + firstCommit_limit) {
    eprintf("No instruction commits for %lu cycles of core %d. Please check the first instruction.\n",
      firstCommit_limit, id);
    eprintf("Note: The first instruction may lie in 0x%lx which may executes and commits after 500 cycles.\n", FIRST_INST_ADDRESS);
    eprintf("   Or the first instruction may lie in 0x%lx which may executes and commits after 2000 cycles.\n", PMEM_BASE);
    display();
    return 1;
  }

  // NOTE: the WFI instruction may cause the CPU to halt for more than `stuck_limit` cycles.
  // We update the `last_commit` if the CPU has a WFI instruction
  // to allow the CPU to run at most `stuck_limit` cycles after WFI resumes execution.
  if (has_wfi()) {
    update_last_commit();
  }

  // check whether there're any commits in the last `stuck_limit` cycles
  if (has_commit && ticks > last_commit + stuck_limit) {
    eprintf("No instruction of core %d commits for %lu cycles, maybe get stuck\n"
        "(please also check whether a fence.i instruction requires more than %lu cycles to flush the icache)\n",
        id, stuck_limit, stuck_limit);
    eprintf("Let REF run one more instruction.\n");
    proxy->exec(1);
    display();
    return 1;
  }

  return 0;
}

void Difftest::raise_trap(int trapCode) {
  dut.trap.valid = 1;
  dut.trap.code = trapCode;
}

void Difftest::clear_step() {
  dut.trap.valid = 0;
  for (int i = 0; i < DIFFTEST_COMMIT_WIDTH; i++) {
    dut.commit[i].valid = 0;
  }
  for (int i = 0; i < DIFFTEST_SBUFFER_RESP_WIDTH; i++) {
    dut.sbuffer[i].resp = 0;
  }
  for (int i = 0; i < DIFFTEST_STORE_WIDTH; i++) {
    dut.store[i].valid = 0;
  }
  for (int i = 0; i < DIFFTEST_COMMIT_WIDTH; i++) {
    dut.load[i].valid = 0;
  }
  for (int i = 0; i < DIFFTEST_PTW_WIDTH; i++) {
    dut.l2tlb[i].valid = 0;
  }
  for (int i = 0; i < DIFFTEST_ITLB_WIDTH; i++) {
    dut.itlb[i].valid = 0;
  }
  for (int i = 0; i < DIFFTEST_LDTLB_WIDTH; i++) {
    dut.ldtlb[i].valid = 0;
  }
  for (int i = 0; i < DIFFTEST_STTLB_WIDTH; i++) {
    dut.sttlb[i].valid = 0;
  }
  dut.atomic.resp = 0;
}

void Difftest::display() {
  state->display(this->id);

  printf("\n==============  REF Regs  ==============\n");
  fflush(stdout);
  proxy->isa_reg_display();
  printf("priviledgeMode: %lu\n", dut.csr.priviledgeMode);
}

void DiffState::display(int coreid) {
  int spike_invalid = test_spike();

  printf("\n============== Commit Group Trace (Core %d) ==============\n", coreid);
  for (int j = 0; j < DEBUG_GROUP_TRACE_SIZE; j++) {
    auto retire_pointer = (retire_group_pointer + DEBUG_GROUP_TRACE_SIZE - 1) % DEBUG_GROUP_TRACE_SIZE;
    printf("commit group [%02d]: pc %010lx cmtcnt %d%s\n",
        j, retire_group_pc_queue[j], retire_group_cnt_queue[j],
        (j == retire_pointer)?" <--" : "");
  }

  printf("\n============== Commit Instr Trace ==============\n");
  for (int j = 0; j < DEBUG_INST_TRACE_SIZE; j++) {
    switch (retire_inst_type_queue[j]) {
      case RET_NORMAL:
        printf("commit inst [%02d]: pc %010lx inst %08x wen %x dst %08x data %016lx robidx %06x%s",
            j, retire_inst_pc_queue[j], retire_inst_inst_queue[j],
            retire_inst_wen_queue[j] != 0, retire_inst_wdst_queue[j],
            retire_inst_wdata_queue[j], retire_inst_robidx_queue[j] , retire_inst_skip_queue[j]?" (skip)":"");
        if(retire_inst_mem_type_queue[j] == RET_LOAD) {
          printf(" lqidx %06x", retire_inst_lqidx_queue[j]);
        }else if(retire_inst_mem_type_queue[j] == RET_STORE) {
          printf(" sqidx %06x", retire_inst_sqidx_queue[j]);
        }else {
          printf("             ");
        }
        break;
      case RET_EXC:
        printf("exception   [%02d]: pc %010lx inst %08x cause %016lx", j,
            retire_inst_pc_queue[j], retire_inst_inst_queue[j], retire_inst_wdata_queue[j]);
        break;
      case RET_INT:
        printf("interrupt   [%02d]: pc %010lx inst %08x cause %016lx", j,
            retire_inst_pc_queue[j], retire_inst_inst_queue[j], retire_inst_wdata_queue[j]);
        break;
    }
    if (!spike_invalid) {
      char inst_str[32];
      char dasm_result[64] = {0};
      sprintf(inst_str, "%08x", retire_inst_inst_queue[j]);
      spike_dasm(dasm_result, inst_str);
      printf(" %s", dasm_result);
    }
    auto retire_pointer = (retire_inst_pointer + DEBUG_INST_TRACE_SIZE - 1) % DEBUG_INST_TRACE_SIZE;
    printf("%s\n", (j == retire_pointer)?" <--" : "");

  }
  fflush(stdout);
}

DiffState::DiffState() {

}
