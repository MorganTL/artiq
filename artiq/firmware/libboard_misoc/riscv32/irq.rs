use riscv::register::{mie, mstatus};
use vexriscv::register::{vmim, vmip};

pub fn enable_interrupts() {
    unsafe{
        mstatus::set_mie();
        mie::set_mext();
    }
}

pub fn disable_interrupts() {
    unsafe {
        mstatus::clear_mie();
        mie::clear_mext();
    }
}

pub fn enable(id: u32) {
    vmim::write(vmim::read() | (1 << id));
}

pub fn disable(id: u32) {
    vmim::write(vmim::read() & !(1 << id));
}

pub fn is_pending(id: u32) -> bool {
    vmip::read() & (1 << id) != 0
}
