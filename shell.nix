{ pkgs ? import <nixpkgs> { } }:
with builtins;
let
  inherit (pkgs) stdenv lib;
  moz_overlay = import (builtins.fetchTarball
    "https://github.com/mozilla/nixpkgs-mozilla/archive/master.tar.gz");
  nixpkgs = import <nixpkgs> { overlays = [ moz_overlay ]; };
  rustBuild = (nixpkgs.rustChannelOf (let
    # Read the ./rust-toolchain (and trim whitespace) so we can extrapolate
    # the channel and date information. This makes it more convenient to
    # update the Rust toolchain used.
    rustToolchain =
      builtins.replaceStrings [ "\n" "\r" " " "	" ] [ "" "" "" "" ]
      (builtins.readFile ./rust-toolchain);
  in {
    channel = lib.head (lib.splitString "-" rustToolchain);
    date =
      lib.concatStringsSep "-" (lib.tail (lib.splitString "-" rustToolchain));
  })).rust.override {
    targets = [ "thumbv7em-none-eabihf" ];
    extensions = [ "rust-src" "llvm-tools-preview" ];
  };
in pkgs.mkShell {
  nativeBuildInputs = with pkgs; [
    rustBuild
    cargo
    cargo-embed
    cargo-flash
    rust-analyzer
    clippy
  ];

  # Certain Rust tools won't work without this
  # This can also be fixed by using oxalica/rust-overlay and specifying the rust-src extension
  # See https://discourse.nixos.org/t/rust-src-not-found-and-other-misadventures-of-developing-rust-on-nixos/11570/3?u=samuela. for more details.
  RUST_SRC_PATH = "${pkgs.rust.packages.stable.rustPlatform.rustLibSrc}";
  NO_RUSTUP = "1";
}
