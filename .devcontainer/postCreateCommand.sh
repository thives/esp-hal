echo "Installing ESP Rust toolchain"

cd $HOME && curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
. $HOME/.cargo/env
cd $HOME && rustup toolchain install stable --component rust-src
cd $HOME && cargo install cargo-make
cd $HOME && rustup target add riscv32imac-unknown-none-elf
cd $HOME && rustup target add aarch64-unknown-linux-musl
cd $HOME && rustup component add rust-analyzer
