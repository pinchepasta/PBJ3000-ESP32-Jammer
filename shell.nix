let
  nixpkgs-ref = "nixos-25.11"; # nixos-25.11
  pyproject-nix-ref = "69f57f27e52a87c54e28138a75ec741cd46663c9";
  zephyr-nix-ref = "6966fb1cbf2fdb494bea3062c5e8e7d44dd8ac9c";
  nixpkgs-esp-dev-ref = "5287d6e1ca9e15ebd5113c41b9590c468e1e001b";

  nixpkgs-esp-dev = builtins.fetchGit {
    url = "https://github.com/mirrexagon/nixpkgs-esp-dev.git";
    rev = nixpkgs-esp-dev-ref;
  };
in

{
  pkgs ? (import (builtins.fetchTarball "https://github.com/NixOS/nixpkgs/archive/${nixpkgs-ref}.tar.gz") {
    overlays = [ (import "${nixpkgs-esp-dev}/overlay.nix") ];
    # The Python library ecdsa is marked as insecure, but we need it for esptool.
    # See https://github.com/mirrexagon/nixpkgs-esp-dev/issues/109
    config.permittedInsecurePackages = [
      "python3.13-ecdsa-0.19.1"
    ];
  }),
  zephyr-src-ref ? "v4.4.0",
  zephyr-sdk-version ? "1_0_1"
}:

let
  zephyr-nix = ((import (builtins.fetchTarball "https://github.com/nix-community/zephyr-nix/archive/${zephyr-nix-ref}.tar.gz")) {
    inherit (pkgs) lib newScope openocd autoreconfHook fetchFromGitHub gcc_multi python312;
    zephyr-src = builtins.fetchTarball "https://github.com/zephyrproject-rtos/zephyr/archive/${zephyr-src-ref}.tar.gz";
    pyproject-nix = (import (builtins.fetchTarball "https://github.com/pyproject-nix/pyproject.nix/archive/${pyproject-nix-ref}.tar.gz")) {
      inherit (pkgs) lib;
    };
  });
  
  zephyr-sdk = zephyr-nix.sdk.override {
    targets = [
      "xtensa-espressif_esp32s3_zephyr-elf"
      "riscv64-zephyr-elf"
    ];
  };
  
  # zephyr-sdk = zephyr-nix.sdkFull;
in
pkgs.mkShell {
  packages = with pkgs; [
    black
    clang-tools
    cmake
    dfu-util
    doxygen
    esptool
    (esp-idf-xtensa.tools.openocd-esp32)
    (gnuradio.override {
      extraPackages = with gnuradioPackages; [
        osmosdr
      ];
      extraPythonPackages = with gnuradio.python.pkgs; [
        numpy
        matplotlib
        pyqt5
      ];
    })
    gqrx
    inspectrum
    librsvg
    libusb1
    mermaid-filter
    ninja
    pandoc
    soapyremote
    soapysdr
    sphinx
    tcpdump
    texliveBasic
    (zephyr-nix.pythonEnv.override {
      extraPackages = ps: with ps; [
        jsonschema
        # Markdown pages in the Sphinx docs (doc/).
        myst-parser
      ];
    })
    zephyr-sdk
  ];

  ZEPHYR_TOOLCHAIN_VARIANT = "zephyr";
  ZEPHYR_SDK_INSTALL_DIR = "${zephyr-sdk}";
}
