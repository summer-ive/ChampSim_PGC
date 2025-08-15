# ChampSim

![GitHub](https://img.shields.io/github/license/ChampSim/ChampSim)
![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/ChampSim/ChampSim/test.yml)
![GitHub forks](https://img.shields.io/github/forks/ChampSim/ChampSim)
[![Coverage Status](https://coveralls.io/repos/github/ChampSim/ChampSim/badge.svg?branch=develop)](https://coveralls.io/github/ChampSim/ChampSim?branch=develop)

ChampSim is a trace-based simulator for a microarchitecture study. If you have questions about how to use ChampSim, we encourage you to search the threads in the Discussions tab or start your own thread. If you are aware of a bug or have a feature request, open a new Issue.

# Using ChampSim

ChampSim is the result of academic research. If you use this software in your work, please cite it using the following reference:

    Gober, N., Chacon, G., Wang, L., Gratz, P. V., Jimenez, D. A., Teran, E., Pugsley, S., & Kim, J. (2022). The Championship Simulator: Architectural Simulation for Education and Competition. https://doi.org/10.48550/arXiv.2210.14324

If you use ChampSim in your work, you may submit a pull request modifying `PUBLICATIONS_USING_CHAMPSIM.bib` to have it featured in [the documentation](https://champsim.github.io/ChampSim/master/Publications-using-champsim.html).

# Download dependencies

ChampSim uses [vcpkg](https://vcpkg.io) to manage its dependencies. In this repository, vcpkg is included as a submodule. You can download the dependencies with

```
git submodule update --init
vcpkg/bootstrap-vcpkg.sh
vcpkg/vcpkg install
```

# Compile

ChampSim takes a JSON configuration script. Examine `champsim_config.json` for a fully-specified example. All options described in this file are optional and will be replaced with defaults if not specified. The configuration scrip can also be run without input, in which case an empty file is assumed.

```
$ ./config.sh <configuration file>
$ make
```

# Download DPC-3 trace

Traces used for the 3rd Data Prefetching Championship (DPC-3) can be found here. (https://dpc3.compas.cs.stonybrook.edu/champsim-traces/speccpu/) A set of traces used for the 2nd Cache Replacement Championship (CRC-2) can be found from this link. (http://bit.ly/2t2nkUj)

Storage for these traces is kindly provided by Daniel Jimenez (Texas A&M University) and Mike Ferdman (Stony Brook University). If you find yourself frequently using ChampSim, it is highly encouraged that you maintain your own repository of traces, in case the links ever break.

# Run simulation

Execute the binary directly.

```
$ bin/champsim --warmup-instructions 200000000 --simulation-instructions 500000000 ~/path/to/traces/600.perlbench_s-210B.champsimtrace.xz
```

The number of warmup and simulation instructions given will be the number of instructions retired. Note that the statistics printed at the end of the simulation include only the simulation phase.

# Add your own branch predictor, data prefetchers, and replacement policy

**Copy an empty template**

```
$ mkdir prefetcher/mypref
$ cp prefetcher/no_l2c/no.cc prefetcher/mypref/mypref.cc
```

**Work on your algorithms with your favorite text editor**

```
$ vim prefetcher/mypref/mypref.cc
```

**Compile and test**
Add your prefetcher to the configuration file.

```
{
    "L2C": {
        "prefetcher": "mypref"
    }
}
```

Note that the example prefetcher is an L2 prefetcher. You might design a prefetcher for a different level.

```
$ ./config.sh <configuration file>
$ make
$ bin/champsim --warmup-instructions 200000000 --simulation-instructions 500000000 600.perlbench_s-210B.champsimtrace.xz
```

# How to create traces

Program traces are available in a variety of locations, however, many ChampSim users wish to trace their own programs for research purposes.
Example tracing utilities are provided in the `tracer/` directory.

# Evaluate Simulation

ChampSim measures the IPC (Instruction Per Cycle) value as a performance metric. <br>
There are some other useful metrics printed out at the end of simulation. <br>

Good luck and be a champion! <br>

# 実行手順

Compile の章 -> Run simulation の章で実行すれば良い。

# Docker コマンド

まず Dockerfile を作成し、基本構成がセットアップされるよう記述する。
現在は下記の設定で Dockerfile を構成し、ChampSim 等はサーバー上にクローンしたものをコンテナにマウントする形で運用している。

```
FROM debian:stable

# Install your favorite packages
# -qq: No output except for errors
RUN apt-get update && \
    # https://anonoz.github.io/tech/2020/04/24/docker-build-stuck-tzdata.html
    # Ubuntu 22.04 doesn't need the two lines below
    DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Tokyo \
    apt-get --yes -qq install \
      build-essential \
      git \
      cmake \
      python3 \
      python3-pip \
      xz-utils \
      wget \
      make \
      curl \
      unzip \
      zip \
      tar \
      parallel \
      pkg-config && \
    rm -rf /var/lib/apt/lists/*

# User/group names/ids which will be overwritten: https://stackoverflow.com/a/44683248
ARG UNAME
ARG GNAME
ARG UID
ARG GID
RUN groupadd -g $GID -o $GNAME
# Add a user
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME

# Set the user (for subsequent commands)
USER $UNAME

# Set the working directory (for subsequent commands)
WORKDIR /home/$UNAME

# Run when the container launches
CMD ["/bin/bash"]
```

したらば Docker イメージを作成。
ホストマシンのファイルは後でマウントする。したがって処理を軽量化するためにビルドコンテキストは dockerfile が入っているディレクトリに指定している。

```
docker build --build-arg UNAME=$(id -un) --build-arg GNAME=$(id -gn) --build-arg UID=$(id -u) --build-arg GID=$(id -g) -t "$(id -un):champsim-pgc.yyyymmdd" -f /home/caras/summerive/dockerfiles/Dockerfile /home/caras/summerive/dockerfiles
```

Docker イメージからコンテナを立ち上げる。
`docker run -it --rm \
  --mount type=bind,source="/home/caras/summerive/ChampSim",target="/home/$(id -un)/ChampSim" \
  --mount type=bind,source="/srv/public/champsim",target="/home/$(id -un)/traces" \
  summerive:champsim-pgc.yyyymmdd`

## ポイント

- `make`よりも`make -j$(nproc)`のほうが並列実行できて早い。(多分)
- `make`でエラーが発生したときは、原因を解決した上で`make clean`を実行してから再度`make`する。

# 変更点

- prefetcher フォルダに signature_path を追加。
- spp の実装で main 関数があるとシミュレータ実行でエラーを吐くので、main 関数を全てコメントアウトした。

# プリフェッチャの解説

- 矢野さんの Slack では SPP の実装にバグがあるとのこと。具体的な内容は不明。
  - 古いバージョンの ChampSim における spp_dev ファイルであるため、現存するバグであるかは不明。
- spp_dev_pgc は PGC を一律で全許可する SPP。
- spp_dev_pgc_adj は、PGC を実行する際に、PGC 先のページが仮想アドレス上で連続するかを確認し、連続性が確認できた場合のみ PGC を実行するもの。
  - 現在実装中。
  - 本来は理想的なシミュレーションに際してこの機能が必要。
    - むやみに全てプリフェッチさせるのはキャッシュ汚染の元になり IPC をむしろ低下させる。
  - ChampSim が仮想アドレスと物理アドレスのマッピングをどのようにシミュレートしているかを調べる必要がある。
    - トレースには物理アドレスのアクセス情報しか載っていない？この LLM の回答が正しいか要検証。
- spp_dev_pgc_grain は、SPP 標準の Signature Table のシグネチャ管理粒度が 4KB なので、このサイズを変更することができるようにしている。

# メモ

- ChampSim の実装は提案論文である"The Championship Simulator: Architectural Simulation for Education and Competition"にある程度記されているため、シミュレータを使う前にこちらを参照するのが懸命かもしれない。
- オリジナルの config は、デフォルトの champsim_config.json を適切にオーバーライドするように書く。
