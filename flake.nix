{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    odri-masterboard-sdk = {
      # FIXME update after https://github.com/open-dynamic-robot-initiative/master-board/pull/173
      url = "git+https://github.com/gwennlbh/master-board?submodules=1&ref=nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    { nixpkgs, odri-masterboard-sdk, ... }:
    let
      pkgs = nixpkgs.legacyPackages.x86_64-linux;
      sdk = odri-masterboard-sdk.packages.x86_64-linux.sdk;
    in
    {
      packages.x86_64-linux.default =
        pkgs.stdenv.mkDerivation rec {
          pname = "odri-control";
          version = "1.0.1";

          src = builtins.path {
            name = pname;
            path = ./.;
          };

          nativeBuildInputs =
            [ sdk ]
            ++ (with pkgs; [
              cmake
              yaml-cpp
              eigen
              python312Packages.eigenpy
              python312Packages.boost
              python312
            ]);
        };
    };
}
