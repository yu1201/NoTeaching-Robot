#!/bin/bash
# NoTeaching-Robot 在线服务部署器（Ubuntu 22.04/24.04）。
#
# 安全约定：
#   * 默认只做 dry-run；真正修改系统必须同时给出 --apply，并交互确认或给出 --yes。
#   * 密码和管理令牌只从权限受限的文件读取，绝不接受命令行明文参数，也绝不打印。
#   * 共享 uploader 永久退役；devicedata 是唯一 full 账号，普通账号必须绑定独立目录。
#   * 管理文件先在同目录写临时文件，再原子替换；失败时从受限备份目录回滚。
#   * 不删除 nginx 的 default/其他站点，不递归改写已有上传数据的属主或权限。
#
# 常用方式：
#   /bin/bash -p deploy_online_services.sh
#   sudo -H /bin/bash -p deploy_online_services.sh --apply --skip-packages
#   sudo -H /bin/bash -p deploy_online_services.sh --apply --yes --skip-packages \
#       --devicedata-password-file /root/secrets/devicedata.password \
#       --enable-ufw --ftp-allow-cidr 10.20.0.0/16 \
#       --data-filesystem-max-bytes 107374182400
#
# 新建 devicedata 时必须提供密码文件。账号已存在时省略对应参数会
# 保留原密码。管理令牌若已存在则原样保留；全新部署会在服务器本机生成。
# 即使操作者误用 bash -x/-v 或继承 xtrace/verbose，也先关闭跟踪，避免后续读取的
# 密码和令牌在参数展开时进入终端、CI 日志或 journal。
set +x +v
set -Eeuo pipefail

# root/生产路径从第一条外部命令起只信任系统目录。离线夹具的根目录须先用可信
# readlink 证明为带 marker 的 canonical 非根目录，之后才允许恢复测试注入的 PATH。
ORIGINAL_PATH="${PATH:-}"
export PATH="/usr/sbin:/usr/bin:/sbin:/bin"
bootstrap_die() {
    builtin printf '错误: %s\n' "$*" >&2
    exit 2
}
if [[ "${DEPLOY_OFFLINE_TEST:-0}" == "1" && -n "${DEPLOY_ROOT:-}" ]]; then
    [[ "$DEPLOY_ROOT" == /* && "$DEPLOY_ROOT" != "/" ]] ||
        bootstrap_die "离线 DEPLOY_ROOT 必须是非 / 的绝对 canonical 目录"
    [[ -d "$DEPLOY_ROOT" && ! -L "$DEPLOY_ROOT" ]] ||
        bootstrap_die "离线 DEPLOY_ROOT 必须是已存在且非符号链接的目录"
    OFFLINE_ROOT_RESOLVED="$(/usr/bin/readlink -f -- "$DEPLOY_ROOT")" ||
        bootstrap_die "无法解析离线 DEPLOY_ROOT"
    [[ "$OFFLINE_ROOT_RESOLVED" == "$DEPLOY_ROOT" && "$OFFLINE_ROOT_RESOLVED" != "/" ]] ||
        bootstrap_die "离线 DEPLOY_ROOT 不得含尾斜杠、重复斜杠、点分量或符号链接"
    OFFLINE_ROOT_MARKER="${DEPLOY_ROOT}/.no-teaching-online-services-offline-root"
    [[ -f "$OFFLINE_ROOT_MARKER" && ! -L "$OFFLINE_ROOT_MARKER" ]] ||
        bootstrap_die "离线 DEPLOY_ROOT 缺少受信测试 marker"
    IFS= read -r OFFLINE_ROOT_MARKER_VALUE < "$OFFLINE_ROOT_MARKER" ||
        bootstrap_die "无法读取离线 DEPLOY_ROOT 测试 marker"
    [[ "$OFFLINE_ROOT_MARKER_VALUE" == "NO_TEACHING_ONLINE_SERVICES_OFFLINE_ROOT_V1" ]] ||
        bootstrap_die "离线 DEPLOY_ROOT 测试 marker 内容无效"
    export PATH="$ORIGINAL_PATH"
fi
unset ORIGINAL_PATH OFFLINE_ROOT_RESOLVED OFFLINE_ROOT_MARKER OFFLINE_ROOT_MARKER_VALUE
unset -f bootstrap_die
SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_PARENT="${SCRIPT_PATH%/*}"
[[ "$SCRIPT_PARENT" != "$SCRIPT_PATH" ]] || SCRIPT_PARENT="."
readonly SCRIPT_NAME="${0##*/}"
readonly SCRIPT_DIR="$(cd -- "$SCRIPT_PARENT" && pwd -P)"
unset SCRIPT_PATH SCRIPT_PARENT

MODE="dry-run"
ASSUME_YES=0
PACKAGES_PREINSTALLED_ACK=0
CONFIGURE_UFW=0
ENABLE_UFW=0
SSH_PORT=48890
PASV_ADDRESS=""
FTP_ALLOW_CIDR=""
DEVICEDATA_PASSWORD_FILE=""
UPLOADER_PASSWORD_FILE=""
DATA_FILESYSTEM_MAX_BYTES=""
ADMIN_TOKEN_SOURCE=""
OTA_ADMIN_SOURCE="${SCRIPT_DIR}/ota_admin.py"

# 仅供仓库离线测试。生产运行不得设置；它也不会绕过 --apply/--yes。
DEPLOY_ROOT="${DEPLOY_ROOT:-}"
DEPLOY_NSS_FIXTURE_DIR="${DEPLOY_NSS_FIXTURE_DIR:-}"
DEPLOY_UFW_RULES_FIXTURE="${DEPLOY_UFW_RULES_FIXTURE:-}"
DEPLOY_CAPACITY_FIXTURE="${DEPLOY_CAPACITY_FIXTURE:-}"
if [[ -n "$DEPLOY_ROOT" && "${DEPLOY_OFFLINE_TEST:-0}" != "1" ]]; then
    echo "错误: DEPLOY_ROOT 仅允许在 DEPLOY_OFFLINE_TEST=1 的离线测试中使用。" >&2
    exit 2
fi
if [[ -n "$DEPLOY_NSS_FIXTURE_DIR" &&
      ( -z "$DEPLOY_ROOT" || "${DEPLOY_OFFLINE_TEST:-0}" != "1" ) ]]; then
    echo "错误: DEPLOY_NSS_FIXTURE_DIR 仅允许隔离离线测试使用。" >&2
    exit 2
fi
if [[ -n "$DEPLOY_UFW_RULES_FIXTURE" &&
      ( -z "$DEPLOY_ROOT" || "${DEPLOY_OFFLINE_TEST:-0}" != "1" ) ]]; then
    echo "错误: DEPLOY_UFW_RULES_FIXTURE 仅允许隔离离线测试使用。" >&2
    exit 2
fi
if [[ -n "$DEPLOY_CAPACITY_FIXTURE" &&
      ( -z "$DEPLOY_ROOT" || "${DEPLOY_OFFLINE_TEST:-0}" != "1" ) ]]; then
    echo "错误: DEPLOY_CAPACITY_FIXTURE 仅允许隔离离线测试使用。" >&2
    exit 2
fi
DEPLOY_ROOT="${DEPLOY_ROOT:-}"

usage() {
    cat <<EOF
用法: ${SCRIPT_NAME} [选项]

默认不修改系统，只输出部署计划。

  --apply                         执行部署（仍需交互确认）
  --yes                           与 --apply 配合，跳过交互确认
  --devicedata-password-file FILE
                                  从 FILE 设置 devicedata 密码；仅新建账号时必需
  --uploader-password-file FILE   已废弃且拒绝：共享 uploader 永久退役
  --admin-token-file FILE         从 FILE 安装管理令牌；省略则保留或本机生成
  --ota-admin-source FILE         ota_admin.py 来源（默认与本脚本同目录）
  --skip-packages                 确认依赖已在隔离维护窗口预装；--apply 必需
  --configure-ufw                 幂等添加私网 FTP 边界（--apply 时必须）
  --enable-ufw                    确保 UFW 为 active（--apply 时必须）
  --ssh-port PORT                 UFW 放行的 SSH 端口（默认 48890）
  --pasv-address HOST_OR_IP       FTP 被动模式公网回址；NAT 服务器应显式提供
  --ftp-allow-cidr CIDR           仅允许该 VPN/受信网段访问 FTP 主/被动端口
  --data-filesystem-max-bytes N   独立 /srv/devicedata 文件系统允许的最大实际容量
  -h, --help                      显示帮助

安全提示：不要把密码或令牌直接放到命令行；秘密文件应由 root 拥有且权限为 0600。
部署器只为本轮新建账号设置初始密码；既有账号改密必须使用 ota-admin 单账号轮换。
既有全局 download/chmod/umask/file-mode 限制不会被静默放宽；冲突时须人工迁移为
每账号 canonical 权限配置后再部署。
--apply 会实测 /srv/devicedata（或 data）是独立挂载且实际总容量不超过 N；
没有这个硬边界会失败。当前仍没有 per-device quota，须另行配置告警和保留阈值。
部署器绝不运行 apt：nginx/vsftpd/ufw/python3/acl 等依赖必须先在独立维护窗口预装。
预装前先用上游 ACL 隔离公网 TCP 21；若做不到，先执行
systemctl mask --runtime --now vsftpd.service，避免 postinst 在 canonical 配置生效前启动 FTP。
安装后确认 vsftpd 为 inactive；若曾 runtime-mask，执行 systemctl unmask --runtime vsftpd.service 后再次确认
仍为 inactive。随后以 --skip-packages 运行部署器，并在核验 UFW 与外部 ACL 后才解除隔离。
EOF
}

die() {
    echo "错误: $*" >&2
    exit 1
}

log() {
    printf '%s\n' "$*"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --apply)
            MODE="apply"
            shift
            ;;
        --yes)
            ASSUME_YES=1
            shift
            ;;
        --devicedata-password-file)
            [[ $# -ge 2 ]] || die "$1 缺少文件路径"
            DEVICEDATA_PASSWORD_FILE="$2"
            shift 2
            ;;
        --uploader-password-file)
            [[ $# -ge 2 ]] || die "$1 缺少文件路径"
            UPLOADER_PASSWORD_FILE="$2"
            shift 2
            ;;
        --data-filesystem-max-bytes)
            [[ $# -ge 2 ]] || die "$1 缺少字节数"
            DATA_FILESYSTEM_MAX_BYTES="$2"
            shift 2
            ;;
        --admin-token-file)
            [[ $# -ge 2 ]] || die "$1 缺少文件路径"
            ADMIN_TOKEN_SOURCE="$2"
            shift 2
            ;;
        --ota-admin-source)
            [[ $# -ge 2 ]] || die "$1 缺少文件路径"
            OTA_ADMIN_SOURCE="$2"
            shift 2
            ;;
        --skip-packages)
            PACKAGES_PREINSTALLED_ACK=1
            shift
            ;;
        --configure-ufw)
            CONFIGURE_UFW=1
            shift
            ;;
        --enable-ufw)
            CONFIGURE_UFW=1
            ENABLE_UFW=1
            shift
            ;;
        --ssh-port)
            [[ $# -ge 2 ]] || die "$1 缺少端口"
            SSH_PORT="$2"
            shift 2
            ;;
        --pasv-address)
            [[ $# -ge 2 ]] || die "$1 缺少主机名或 IPv4 地址"
            PASV_ADDRESS="$2"
            shift 2
            ;;
        --ftp-allow-cidr)
            [[ $# -ge 2 ]] || die "$1 缺少 IPv4/IPv6 网络 CIDR"
            FTP_ALLOW_CIDR="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            [[ $# -eq 0 ]] || die "不接受位置参数或明文秘密"
            ;;
        *)
            die "未知参数 '$1'；本脚本不接受明文 FTP 密码"
            ;;
    esac
done

[[ "$SSH_PORT" =~ ^[0-9]+$ ]] && (( SSH_PORT >= 1 && SSH_PORT <= 65535 )) ||
    die "--ssh-port 必须是 1-65535"
[[ -z "$DATA_FILESYSTEM_MAX_BYTES" ||
   ( "$DATA_FILESYSTEM_MAX_BYTES" =~ ^[1-9][0-9]*$ &&
     ${#DATA_FILESYSTEM_MAX_BYTES} -le 20 ) ]] ||
    die "--data-filesystem-max-bytes 必须是 1-20 位正整数字节数"
[[ -z "$UPLOADER_PASSWORD_FILE" ]] ||
    die "共享 uploader 已永久退役；禁止再提供 --uploader-password-file"
if [[ -n "$PASV_ADDRESS" ]]; then
    [[ ${#PASV_ADDRESS} -le 253 && "$PASV_ADDRESS" =~ ^[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?$ &&
       "$PASV_ADDRESS" != *..* ]] ||
        die "--pasv-address 必须是单个有效主机名或 IPv4 地址"
fi
if [[ "$CONFIGURE_UFW" -eq 1 && -z "$FTP_ALLOW_CIDR" ]]; then
    die "--configure-ufw/--enable-ufw 必须显式提供 --ftp-allow-cidr，拒绝把 FTP 暴露到公网"
fi
if [[ -n "$FTP_ALLOW_CIDR" ]]; then
    command -v python3 >/dev/null 2>&1 || die "CIDR 校验需要 python3"
    python3 -I - "$FTP_ALLOW_CIDR" <<'PY'
import ipaddress
import sys

try:
    network = ipaddress.ip_network(sys.argv[1], strict=True)
except ValueError as exc:
    raise SystemExit(f"无效或含主机位的 --ftp-allow-cidr: {exc}")
allowed = tuple(
    ipaddress.ip_network(value)
    for value in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16", "100.64.0.0/10", "fc00::/7")
)
if not any(network.version == parent.version and network.subnet_of(parent) for parent in allowed):
    raise SystemExit("--ftp-allow-cidr 仅允许 RFC1918、CGNAT 100.64/10 或 IPv6 ULA 子网")
PY
fi

if [[ "$MODE" != "apply" && "$ASSUME_YES" -eq 1 ]]; then
    die "--yes 只能与 --apply 配合使用"
fi
if [[ "$MODE" == "apply" && "$PACKAGES_PREINSTALLED_ACK" -ne 1 ]]; then
    die "--apply 必须显式给出 --skip-packages，确认依赖已在隔离维护窗口预装；部署器绝不运行 apt"
fi
if [[ "$MODE" == "apply" ]]; then
    [[ "$CONFIGURE_UFW" -eq 1 && "$ENABLE_UFW" -eq 1 && -n "$FTP_ALLOW_CIDR" ]] ||
        die "明文 FTP 生产部署必须给出 --enable-ufw 和私网/VPN --ftp-allow-cidr"
    [[ -n "$DATA_FILESYSTEM_MAX_BYTES" ]] ||
        die "--apply 必须给出 --data-filesystem-max-bytes，并通过独立受限文件系统实测"
fi
if [[ "$MODE" == "apply" && -z "$DEPLOY_ROOT" ]]; then
    [[ "$EUID" -eq 0 ]] || die "--apply 必须由 root 执行"
fi

# 必须早于 staging、备份、锁文件及任何系统写入。--skip-packages 只是操作者对
# 外部维护窗口的显式确认，并不会跳过这里对实际可执行依赖的 fail-closed 校验。
log "=== 1/7 验证外部维护窗口已预装依赖（部署器绝不运行 apt） ==="
for command_name in awk basename cat chmod chown cp date dirname find grep install mkdir \
    mktemp mv python3 readlink rm sh sha256sum stat tail tr; do
    command -v "$command_name" >/dev/null 2>&1 || die "缺少预装命令: $command_name"
done
if [[ -z "$DEPLOY_ROOT" ]]; then
    for command_name in chpasswd findmnt flock getent gpasswd groupadd groupdel id nginx \
        systemctl timeout useradd userdel usermod vsftpd; do
        command -v "$command_name" >/dev/null 2>&1 ||
            die "缺少预装系统依赖: $command_name；请退出部署并在隔离维护窗口安装"
    done
    if [[ "$CONFIGURE_UFW" -eq 1 ]]; then
        command -v ufw >/dev/null 2>&1 ||
            die "请求配置防火墙但 ufw 未预装；请在隔离维护窗口安装"
    fi
    if [[ "$ENABLE_UFW" -eq 1 ]]; then
        command -v ss >/dev/null 2>&1 || die "请求启用防火墙但缺少预装命令: ss"
    fi
fi

target_path() {
    printf '%s%s' "$DEPLOY_ROOT" "$1"
}

readonly NGINX_SITE="$(target_path /etc/nginx/sites-available/ota)"
readonly NGINX_LINK="$(target_path /etc/nginx/sites-enabled/ota)"
readonly VSFTPD_CONFIG="$(target_path /etc/vsftpd.conf)"
readonly VSFTPD_USERLIST="$(target_path /etc/vsftpd.userlist)"
readonly VSFTPD_USER_CONF_DIR="$(target_path /etc/vsftpd_user_conf)"
readonly SHELLS_FILE="$(target_path /etc/shells)"
readonly OTA_ADMIN_DIR="$(target_path /opt/ota-admin)"
readonly OTA_ADMIN_TARGET="$(target_path /opt/ota-admin/ota_admin.py)"
readonly OTA_ADMIN_TOKEN="$(target_path /opt/ota-admin/token)"
readonly OTA_ADMIN_UNIT="$(target_path /etc/systemd/system/ota-admin.service)"
readonly CLEANUP_CRON="$(target_path /etc/cron.daily/clean-devicedata)"
readonly FTP_ROOT="$(target_path /srv/devicedata)"
readonly FTP_DATA="$(target_path /srv/devicedata/data)"
readonly OTA_ROOT="$(target_path /var/www/ota)"
readonly OFFLINE_ACCOUNT_STATE="$(target_path /var/lib/no-teaching-online-services/test-accounts)"
readonly ACCOUNT_LOCK_DIR="/run/no-teaching-ota"
readonly ACCOUNT_LOCK_FILE="${ACCOUNT_LOCK_DIR}/ota-accounts.lock"

CAPACITY_STATUS="--apply 时必须实测独立受限文件系统"
validate_capacity_boundary() {
    local fixture_target fixture_total fixture_source fixture_identity fixture_parent_identity
    local fixture_fstype fixture_options query mount_target mount_source mount_identity mount_fstype mount_options
    local stat_values block_size block_count mount_device parent_device parent_path
    if [[ -n "$DEPLOY_ROOT" ]]; then
        [[ -f "$DEPLOY_CAPACITY_FIXTURE" && ! -L "$DEPLOY_CAPACITY_FIXTURE" ]] ||
            die "离线 --apply 必须提供普通文件 DEPLOY_CAPACITY_FIXTURE"
        IFS=$'\t' read -r fixture_target fixture_total fixture_source fixture_identity \
            fixture_parent_identity fixture_fstype fixture_options < <(python3 -I - "$DEPLOY_CAPACITY_FIXTURE" <<'PY'
import json
import re
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as stream:
    value = json.load(stream)
required = {
    "mountTarget", "totalBytes", "mountSource", "mountIdentity",
    "parentIdentity", "fsType", "mountOptions",
}
if set(value) != required:
    raise SystemExit("容量夹具字段不完整或含未知字段")
target = value["mountTarget"]
total = value["totalBytes"]
if target not in ("/srv/devicedata", "/srv/devicedata/data"):
    raise SystemExit("容量夹具 mountTarget 必须是独立设备数据挂载点")
if isinstance(total, bool) or not isinstance(total, int) or total <= 0:
    raise SystemExit("容量夹具 totalBytes 必须是正整数")
for key in required - {"totalBytes"}:
    if not isinstance(value[key], str) or not value[key] or any(ch in value[key] for ch in "\r\n\t"):
        raise SystemExit(f"容量夹具 {key} 必须是单行非空字符串")
if value["fsType"] not in {"ext4", "xfs"}:
    raise SystemExit("容量夹具只接受独立 ext4/xfs；btrfs 子卷须先实现并实测 qgroup quota")
if not value["mountSource"].startswith("/dev/"):
    raise SystemExit("容量夹具 mountSource 必须是独立块设备")
if not re.fullmatch(r"[0-9]+:[0-9]+", value["mountIdentity"]) or not re.fullmatch(
    r"[0-9]+:[0-9]+", value["parentIdentity"]
):
    raise SystemExit("容量夹具 mountIdentity/parentIdentity 格式无效")
if value["mountIdentity"] == value["parentIdentity"]:
    raise SystemExit("容量夹具显示数据目录与父目录同一文件系统（bind/共享盘）")
options = set(value["mountOptions"].split(","))
if "bind" in options or "rbind" in options:
    raise SystemExit("容量夹具显示 bind/rbind，不能隔离磁盘耗尽")
print(
    target, total, value["mountSource"], value["mountIdentity"],
    value["parentIdentity"], value["fsType"], value["mountOptions"], sep="\t"
)
PY
        ) || die "无法验证离线容量夹具"
        fixture_options="${fixture_options//$'\r'/}"
        [[ "$fixture_total" -le "$DATA_FILESYSTEM_MAX_BYTES" ]] ||
            die "独立数据文件系统实际容量 ${fixture_total} 超过声明上限 ${DATA_FILESYSTEM_MAX_BYTES}"
        CAPACITY_STATUS="离线夹具证明 ${fixture_target}(${fixture_source},${fixture_identity},${fixture_fstype}) 独立容量=${fixture_total} bytes"
        return
    fi

    # 生产前置条件由外部存储维护窗口完成。部署器不创建 loop/LVM，也不把一个
    # “已确认”开关当配额；它只接受 findmnt/stat 对当前内核挂载状态的实测结果。
    if [[ -d /srv/devicedata/data && ! -L /srv/devicedata/data ]]; then
        query=/srv/devicedata/data
    elif [[ -d /srv/devicedata && ! -L /srv/devicedata ]]; then
        query=/srv/devicedata
    else
        die "容量硬门禁要求预先挂载独立 /srv/devicedata（或 /srv/devicedata/data）文件系统"
    fi
    mount_target="$(findmnt --raw --noheadings --output TARGET --target "$query")" ||
        die "findmnt 无法解析设备数据挂载点"
    [[ "$mount_target" != *$'\n'* &&
       ( "$mount_target" == "/srv/devicedata" || "$mount_target" == "/srv/devicedata/data" ) ]] ||
        die "设备数据仍落在共享文件系统 ${mount_target:-未知}；必须使用独立受限挂载"
    [[ "$(readlink -f -- "$mount_target")" == "$mount_target" ]] ||
        die "设备数据挂载点不得经过符号链接或非规范路径"
    mount_source="$(findmnt --raw --noheadings --output SOURCE --target "$query")" ||
        die "无法回读设备数据挂载源"
    mount_identity="$(findmnt --raw --noheadings --output MAJ:MIN --target "$query")" ||
        die "无法回读设备数据挂载设备号"
    mount_fstype="$(findmnt --raw --noheadings --output FSTYPE --target "$query")" ||
        die "无法回读设备数据文件系统类型"
    mount_options="$(findmnt --raw --noheadings --output OPTIONS --target "$query")" ||
        die "无法回读设备数据挂载选项"
    [[ "$mount_source" == /dev/* && "$mount_identity" =~ ^[0-9]+:[0-9]+$ ]] ||
        die "设备数据挂载必须来自可审计的独立块设备"
    [[ "$mount_fstype" == "ext4" || "$mount_fstype" == "xfs" ]] ||
        die "只接受独立 ext4/xfs；btrfs 子卷须先实现并实测 qgroup quota"
    [[ ",${mount_options}," != *,bind,* && ",${mount_options}," != *,rbind,* ]] ||
        die "bind/rbind 挂载与父目录共享容量，拒绝冒充独立上限"
    parent_path="$(dirname -- "$mount_target")"
    mount_device="$(stat -c '%d' -- "$mount_target")" || die "无法回读数据挂载设备身份"
    parent_device="$(stat -c '%d' -- "$parent_path")" || die "无法回读父目录设备身份"
    [[ "$mount_device" != "$parent_device" ]] ||
        die "数据挂载与父目录 st_dev 相同，无法排除 bind/同源共享文件系统"
    stat_values="$(stat -f -c '%S:%b' -- "$mount_target")" ||
        die "无法实测设备数据文件系统容量"
    IFS=: read -r block_size block_count <<< "$stat_values"
    fixture_total="$(python3 -I - "$block_size" "$block_count" "$DATA_FILESYSTEM_MAX_BYTES" <<'PY'
import sys
block_size, block_count, expected_max = (int(value) for value in sys.argv[1:])
actual = block_size * block_count
if block_size <= 0 or block_count <= 0 or actual > expected_max:
    raise SystemExit(
        f"独立数据文件系统实际容量 {actual} 超过声明上限 {expected_max}"
    )
print(actual)
PY
    )" || die "独立数据文件系统不满足声明容量上限"
    CAPACITY_STATUS="${mount_target}(${mount_source},${mount_identity},${mount_fstype}) 独立容量=${fixture_total} bytes（上限 ${DATA_FILESYSTEM_MAX_BYTES}）"
}

if [[ "$MODE" == "apply" ]]; then
    validate_capacity_boundary
fi

for regular_target in "$NGINX_SITE" "$VSFTPD_CONFIG" "$VSFTPD_USERLIST" \
    "$SHELLS_FILE" "$OTA_ADMIN_TARGET" "$OTA_ADMIN_TOKEN" "$OTA_ADMIN_UNIT" \
    "$CLEANUP_CRON" "${VSFTPD_USER_CONF_DIR}/uploader" \
    "${VSFTPD_USER_CONF_DIR}/devicedata"; do
    [[ ! -L "$regular_target" ]] || die "敏感目标不能是符号链接: $regular_target"
    [[ ! -e "$regular_target" || -f "$regular_target" ]] ||
        die "敏感目标必须是普通文件: $regular_target"
done

lookup_passwd_entry() {
    local account="$1"
    if [[ -n "$DEPLOY_NSS_FIXTURE_DIR" ]]; then
        awk -F: -v wanted="$account" '$1 == wanted { print }' \
            "${DEPLOY_NSS_FIXTURE_DIR}/passwd"
    else
        getent passwd "$account"
    fi
}

all_passwd_entries() {
    if [[ -n "$DEPLOY_NSS_FIXTURE_DIR" ]]; then
        cat -- "${DEPLOY_NSS_FIXTURE_DIR}/passwd"
    else
        getent passwd
    fi
}

lookup_group_entry() {
    local group="$1"
    if [[ -n "$DEPLOY_NSS_FIXTURE_DIR" ]]; then
        awk -F: -v wanted="$group" '$1 == wanted { print }' \
            "${DEPLOY_NSS_FIXTURE_DIR}/group"
    else
        getent group "$group"
    fi
}

all_group_entries() {
    if [[ -n "$DEPLOY_NSS_FIXTURE_DIR" ]]; then
        cat -- "${DEPLOY_NSS_FIXTURE_DIR}/group"
    else
        getent group
    fi
}

account_exists() {
    local account="$1"
    if [[ -n "$DEPLOY_ROOT" ]]; then
        [[ -f "$OFFLINE_ACCOUNT_STATE" ]] && grep -Fqx -- "$account" "$OFFLINE_ACCOUNT_STATE"
    else
        id "$account" >/dev/null 2>&1
    fi
}

validate_existing_ftp_account() {
    local account="$1" entry name password uid gid gecos home shell extra matching_names
    entry="$(lookup_passwd_entry "$account")" || die "无法读取既有账号 ${account} 的 passwd 信息"
    [[ "$entry" != *$'\n'* ]] || die "既有账号 ${account} 的 passwd 记录不唯一"
    IFS=: read -r name password uid gid gecos home shell extra <<< "$entry"
    [[ "$name" == "$account" && -z "${extra:-}" ]] ||
        die "既有账号 ${account} 的 passwd 记录格式异常"
    [[ "$uid" =~ ^[0-9]+$ && "$uid" -ge 1000 ]] ||
        die "既有账号 ${account} 的 UID 必须 >=1000，拒绝系统/root UID 别名"
    matching_names="$(all_passwd_entries | awk -F: -v wanted_uid="$uid" \
        '$3 == wanted_uid { print $1 }')"
    [[ "$matching_names" == "$account" ]] ||
        die "账号 ${account} 的 UID=${uid} 被其他用户名共享或无法唯一确认"
    [[ "$home" == "/srv/devicedata" ]] ||
        die "既有账号 ${account} 的 home 不是 /srv/devicedata，拒绝白名单化"
    [[ "$shell" == "/usr/sbin/nologin" ]] ||
        die "既有账号 ${account} 的 shell 不是 /usr/sbin/nologin，拒绝白名单化"
}

validate_ftpdata_group() {
    local entry name password gid members extra matching_names
    entry="$(lookup_group_entry ftpdata)" || die "无法读取 ftpdata 组"
    [[ "$entry" != *$'\n'* ]] || die "ftpdata 组记录不唯一"
    IFS=: read -r name password gid members extra <<< "$entry"
    [[ "$name" == "ftpdata" && -z "${extra:-}" && "$gid" =~ ^[0-9]+$ && "$gid" -gt 0 ]] ||
        die "ftpdata 必须使用唯一的非 root GID"
    matching_names="$(all_group_entries | awk -F: -v wanted_gid="$gid" \
        '$3 == wanted_gid { print $1 }')"
    [[ "$matching_names" == "ftpdata" ]] ||
        die "ftpdata 的 GID=${gid} 被其他组别名占用或无法唯一确认"
}

validate_devicedata_private_group() {
    local passwd_entry admin_uid admin_gid group_entry group_name group_members aliases users_with_gid ftp_entry ftp_gid
    passwd_entry="$(lookup_passwd_entry devicedata)" || die "无法读取 devicedata 身份"
    IFS=: read -r _ _ admin_uid admin_gid _ _ _ _ <<< "$passwd_entry"
    [[ "$admin_uid" =~ ^[0-9]+$ && "$admin_uid" -ge 1000 &&
       "$admin_gid" =~ ^[0-9]+$ && "$admin_gid" -gt 0 ]] ||
        die "devicedata 必须使用非特权 UID 和私有主 GID"
    group_entry="$(all_group_entries | awk -F: -v gid="$admin_gid" '$3 == gid { print }')"
    [[ -n "$group_entry" && "$group_entry" != *$'\n'* ]] ||
        die "devicedata 主 GID=${admin_gid} 未唯一映射到私有组"
    IFS=: read -r group_name _ _ group_members _ <<< "$group_entry"
    aliases="$(all_group_entries | awk -F: -v gid="$admin_gid" '$3 == gid { print $1 }')"
    [[ "$aliases" == "$group_name" ]] || die "devicedata 私有 GID 存在组别名"
    [[ -z "$group_members" || "$group_members" == "devicedata" ]] ||
        die "devicedata 私有组含其它成员，无法隔离普通设备目录"
    users_with_gid="$(all_passwd_entries | awk -F: -v gid="$admin_gid" '$4 == gid { print $1 }')"
    [[ "$users_with_gid" == "devicedata" ]] ||
        die "devicedata 主 GID 被其它账号共享，无法作为管理访问边界"
    ftp_entry="$(lookup_group_entry ftpdata)" || die "无法读取 ftpdata 组"
    IFS=: read -r _ _ ftp_gid _ _ <<< "$ftp_entry"
    [[ "$admin_gid" != "$ftp_gid" ]] ||
        die "devicedata 主组不能是共享 ftpdata；请先迁移为私有主组"
}

validate_device_private_group() {
    local account="$1" passwd_entry primary_gid admin_entry admin_gid ftp_entry ftp_gid group_entry group_name members aliases users
    passwd_entry="$(lookup_passwd_entry "$account")" || die "无法读取设备账号 ${account} 身份"
    IFS=: read -r _ _ _ primary_gid _ _ _ _ <<< "$passwd_entry"
    admin_entry="$(lookup_passwd_entry devicedata)" || die "无法读取 devicedata 身份"
    IFS=: read -r _ _ _ admin_gid _ _ _ _ <<< "$admin_entry"
    ftp_entry="$(lookup_group_entry ftpdata)" || die "无法读取 ftpdata 组"
    IFS=: read -r _ _ ftp_gid _ _ <<< "$ftp_entry"
    [[ "$primary_gid" != "$ftp_gid" && "$primary_gid" != "$admin_gid" ]] ||
        die "设备账号 ${account} 必须使用独立私有主组"
    group_entry="$(all_group_entries | awk -F: -v gid="$primary_gid" '$3 == gid { print }')"
    [[ -n "$group_entry" && "$group_entry" != *$'\n'* ]] ||
        die "设备账号 ${account} 主 GID 未唯一映射"
    IFS=: read -r group_name _ _ members _ <<< "$group_entry"
    aliases="$(all_group_entries | awk -F: -v gid="$primary_gid" '$3 == gid { print $1 }')"
    users="$(all_passwd_entries | awk -F: -v gid="$primary_gid" '$4 == gid { print $1 }')"
    [[ "$group_name" == "$account" && "$aliases" == "$account" && "$users" == "$account" &&
       ( -z "$members" || "$members" == "$account" ) ]] ||
        die "设备账号 ${account} 的私有主组被共享、别名化或含其它成员"
}

validate_ftpdata_membership_closure() {
    local entry name password gid members extra passwd_line p_name p_gid member
    local -a supplementary=()
    local -A seen_members=()
    entry="$(lookup_group_entry ftpdata)" || die "无法读取 ftpdata 组成员"
    IFS=: read -r name password gid members extra <<< "$entry"
    IFS=, read -r -a supplementary <<< "$members"
    for member in "${supplementary[@]}"; do
        [[ -n "$member" ]] && seen_members["$member"]=1
    done
    while IFS= read -r passwd_line || [[ -n "$passwd_line" ]]; do
        IFS=: read -r p_name _ _ p_gid _ _ _ <<< "$passwd_line"
        [[ "$p_gid" == "$gid" && -n "$p_name" ]] && seen_members["$p_name"]=1
    done < <(all_passwd_entries)

    for member in "${!seen_members[@]}"; do
        if [[ "$member" == "uploader" ]]; then
            ! grep -Fqx -- uploader "${STAGE_DIR}/vsftpd.userlist" ||
                die "共享 uploader 已退役，禁止保留在最终 FTP 白名单"
            continue
        fi
        grep -Fqx -- "$member" "${STAGE_DIR}/vsftpd.userlist" ||
            die "ftpdata 成员 ${member} 未在最终 vsftpd.userlist，拒绝隐藏组权限"
        validate_existing_ftp_account "$member"
    done
}

user_in_ftpdata() {
    local account="$1" group_entry group_gid group_members passwd_entry primary_gid
    if [[ -n "$DEPLOY_NSS_FIXTURE_DIR" ]]; then
        group_entry="$(lookup_group_entry ftpdata)" || return 1
        IFS=: read -r _ _ group_gid group_members _ <<< "$group_entry"
        if tr ',' '\n' <<< "$group_members" | grep -Fqx -- "$account"; then
            return 0
        fi
        passwd_entry="$(lookup_passwd_entry "$account")" || return 1
        IFS=: read -r _ _ _ primary_gid _ _ _ <<< "$passwd_entry"
        [[ "$primary_gid" == "$group_gid" ]]
    else
        id -nG "$account" | tr ' ' '\n' | grep -Fqx ftpdata
    fi
}

mark_ftpdata_membership_added() {
    case "$1" in
        devicedata) ADDED_DEVICEDATA_TO_FTPDATA=1 ;;
        *) die "内部错误: 未知 FTP 账号 $1" ;;
    esac
}

add_existing_account_to_ftpdata() {
    local account="$1" rc
    user_in_ftpdata "$account" && return 0

    # timeout 可能在 usermod 已写入 /etc/group 后才返回非零；无论退出码如何都必须
    # 重新读取最终成员关系，并在已提交时登记回滚责任。
    set +e
    timeout --signal=TERM --kill-after=5s 30s usermod -a -G ftpdata "$account"
    rc=$?
    set -e
    if [[ $rc -eq 0 ]] || user_in_ftpdata "$account"; then
        mark_ftpdata_membership_added "$account"
    fi
    [[ $rc -eq 0 ]] || die "将 ${account} 加入 ftpdata 失败（退出码 ${rc}）"
    user_in_ftpdata "$account" || die "usermod 成功返回但 ${account} 未加入 ftpdata"
}

create_ftpdata_group() {
    local rc
    lookup_group_entry ftpdata >/dev/null 2>&1 && die "内部错误: ftpdata 已存在却进入创建路径"
    # 先登记回滚责任，关闭外部命令成功与下一条赋值之间的 TERM/INT 窗口。
    CREATED_FTP_GROUP=1
    set +e
    timeout --signal=TERM --kill-after=5s 30s groupadd --system ftpdata
    rc=$?
    set -e
    if ! lookup_group_entry ftpdata >/dev/null 2>&1; then
        CREATED_FTP_GROUP=0
        die "创建 ftpdata 失败（退出码 ${rc}）"
    fi
    [[ $rc -eq 0 ]] || die "groupadd 结果不确定（退出码 ${rc}），进入回滚"
}

create_ftp_account() {
    local account="$1" rc
    id "$account" >/dev/null 2>&1 && die "内部错误: ${account} 已存在却进入创建路径"
    case "$account" in
        devicedata) CREATED_DEVICEDATA=1 ;;
        *) die "内部错误: 不允许创建账号 ${account}" ;;
    esac
    set +e
    timeout --signal=TERM --kill-after=5s 30s \
        useradd -U -M -d /srv/devicedata -s /usr/sbin/nologin -G ftpdata "$account"
    rc=$?
    set -e
    if ! id "$account" >/dev/null 2>&1; then
        case "$account" in
            devicedata) CREATED_DEVICEDATA=0 ;;
        esac
        die "创建 ${account} 失败（退出码 ${rc}）"
    fi
    [[ $rc -eq 0 ]] || die "useradd ${account} 结果不确定（退出码 ${rc}），进入回滚"
}

validate_ssh_listener_before_ufw_enable() {
    local active_port extra
    [[ -n "${SSH_CONNECTION:-}" ]] ||
        die "--enable-ufw 要求 SSH_CONNECTION，以确认不会锁死当前远程会话"
    IFS=' ' read -r _ _ _ active_port extra <<< "$SSH_CONNECTION"
    [[ -n "$active_port" && -z "${extra:-}" && "$active_port" == "$SSH_PORT" ]] ||
        die "--ssh-port=${SSH_PORT} 与当前 SSH_CONNECTION 本地端口不一致"
    ss -H -ltn | awk -v suffix=":${SSH_PORT}" '
        $4 ~ (suffix "$") { found = 1 }
        END { exit(found ? 0 : 1) }
    ' || die "--ssh-port=${SSH_PORT} 没有可确认的 TCP 监听，拒绝自动启用 UFW"
}

validate_ufw_rules_file() {
    local rules_file="$1"
    [[ -f "$rules_file" && ! -L "$rules_file" ]] || die "UFW 审计输入不是普通文件"
    python3 -I - "$FTP_ALLOW_CIDR" "$rules_file" <<'PY'
import shlex
import sys

cidr, path = sys.argv[1:]
ftp_ranges = ((21, 21), (40000, 40100))
accepted = {
    ("ufw", "allow", "from", cidr, "to", "any", "port", "21", "proto", "tcp"),
    ("ufw", "allow", "from", cidr, "to", "any", "port", "40000:40100", "proto", "tcp"),
}


def parse_numeric_port(spec):
    lower = spec.lower()
    protocol = None
    if "/" in lower:
        lower, protocol = lower.rsplit("/", 1)
    separator = ":" if ":" in lower else ("-" if "-" in lower else None)
    try:
        if separator:
            start, end = (int(value) for value in lower.split(separator, 1))
        else:
            start = end = int(lower)
    except ValueError:
        return None
    if not (1 <= start <= end <= 65535):
        return None
    return start, end, protocol


def overlaps_ftp(start, end):
    return any(start <= ftp_end and ftp_start <= end for ftp_start, ftp_end in ftp_ranges)


for raw in open(path, "r", encoding="utf-8", errors="strict"):
    line = raw.strip()
    if not line.startswith("ufw "):
        continue
    try:
        tokens = shlex.split(line)
    except ValueError:
        raise SystemExit("现有 UFW 规则无法安全解析，请人工审计")
    lower_tokens = [token.lower() for token in tokens]
    permit_indexes = [i for i, token in enumerate(lower_tokens) if token in {"allow", "limit"}]
    if not permit_indexes:
        continue
    if len(permit_indexes) != 1:
        raise SystemExit("现有 UFW allow 规则无法安全解析，请人工审计")
    action_index = permit_indexes[0]
    port_specs = []
    for i, token in enumerate(lower_tokens[:-1]):
        if token == "port":
            port_specs.append(tokens[i + 1])
    if not port_specs and action_index + 1 < len(tokens):
        candidate = lower_tokens[action_index + 1]
        if candidate not in {"from", "to", "in", "out", "on", "proto", "comment"}:
            port_specs.append(tokens[action_index + 1])
    if len(port_specs) != 1:
        raise SystemExit("现有 UFW allow 规则未限定可证明安全的单一端口")
    parsed = parse_numeric_port(port_specs[0])
    if parsed is None:
        raise SystemExit("现有 UFW allow 使用应用 profile/服务名或未知端口，无法证明不含 FTP")
    start, end, suffix_protocol = parsed
    protocol = suffix_protocol
    if "proto" in lower_tokens:
        index = lower_tokens.index("proto")
        if index + 1 >= len(tokens):
            raise SystemExit("现有 UFW proto 规则格式无效")
        declared = lower_tokens[index + 1]
        if protocol is not None and protocol != declared:
            raise SystemExit("现有 UFW 协议声明冲突")
        protocol = declared
    if protocol not in {None, "tcp", "udp"}:
        raise SystemExit("现有 UFW 协议无法证明安全")
    if protocol == "udp" or not overlaps_ftp(start, end):
        continue
    if tuple(tokens) not in accepted:
        raise SystemExit("发现更宽或无法证明等价的旧 FTP UFW 规则；请人工删除后重试")
PY
}

audit_existing_ufw_ftp_rules() {
    local rules_file="${STAGE_DIR}/ufw-added-rules"
    LC_ALL=C ufw show added > "$rules_file" || die "无法审计现有 UFW 规则"
    validate_ufw_rules_file "$rules_file"
}

verify_final_private_ftp_firewall() {
    local rules_file="${STAGE_DIR}/ufw-final-added-rules"
    local status_file="${STAGE_DIR}/ufw-final-status"
    LC_ALL=C ufw status verbose > "$status_file" || die "无法回读最终 UFW 状态"
    grep -Fqx 'Status: active' "$status_file" ||
        die "最终 UFW 不是 active，明文 FTP 边界未生效"
    grep -Eq '^Default: deny \(incoming\),' "$status_file" ||
        die "最终 UFW 默认入站策略不是 deny，私网 FTP allow 不能形成边界"
    LC_ALL=C ufw show added > "$rules_file" || die "无法回读最终 UFW 规则"
    validate_ufw_rules_file "$rules_file"
    grep -Fqx "ufw allow from ${FTP_ALLOW_CIDR} to any port 21 proto tcp" "$rules_file" ||
        die "最终 UFW 缺少 FTP 控制端口私网规则"
    grep -Fqx "ufw allow from ${FTP_ALLOW_CIDR} to any port 40000:40100 proto tcp" "$rules_file" ||
        die "最终 UFW 缺少 FTP 被动端口私网规则"
}

ensure_private_ftp_firewall() {
    [[ -f /etc/default/ufw && ! -L /etc/default/ufw ]] ||
        die "缺少可信 /etc/default/ufw，无法证明启用后的默认入站策略"
    grep -Fqx 'DEFAULT_INPUT_POLICY="DROP"' /etc/default/ufw ||
        die "UFW DEFAULT_INPUT_POLICY 必须预先设为 DROP；部署器不擅自改写全局策略"
    audit_existing_ufw_ftp_rules
    # SSH 规则先落地；FTP 永远只接收经严格 CIDR 校验的私网/VPN 来源。
    ufw allow "${SSH_PORT}/tcp"
    ufw allow 8090/tcp
    ufw allow from "$FTP_ALLOW_CIDR" to any port 21 proto tcp
    ufw allow from "$FTP_ALLOW_CIDR" to any port 40000:40100 proto tcp
    if ! LC_ALL=C ufw status | grep -Fqx 'Status: active'; then
        validate_ssh_listener_before_ufw_enable
        ufw --force enable
    fi
    verify_final_private_ftp_firewall
}

account_lock_directory_is_secure() {
    local attributes mode uid gid
    [[ -d "$ACCOUNT_LOCK_DIR" && ! -L "$ACCOUNT_LOCK_DIR" ]] || return 1
    attributes="$(stat -c '%a:%u:%g' -- "$ACCOUNT_LOCK_DIR")" || return 1
    IFS=: read -r mode uid gid <<< "$attributes"
    [[ "$mode" == "700" && "$uid" == "0" && "$gid" == "0" ]]
}

prepare_account_lock_directory() {
    local run_attributes run_mode run_uid run_gid
    [[ -d /run && ! -L /run ]] || die "账号事务锁父目录 /run 不安全"
    run_attributes="$(stat -c '%a:%u:%g' -- /run)" || die "无法审计 /run"
    IFS=: read -r run_mode run_uid run_gid <<< "$run_attributes"
    [[ "$run_uid" == "0" && "$run_gid" == "0" &&
       $((8#$run_mode & 8#022)) -eq 0 ]] ||
        die "/run 必须由 root:root 拥有且不可由 group/other 写"
    if [[ ! -e "$ACCOUNT_LOCK_DIR" ]]; then
        install -d -m 0700 -o root -g root -- "$ACCOUNT_LOCK_DIR"
    fi
    account_lock_directory_is_secure ||
        die "账号事务锁目录必须为 root:root 0700: $ACCOUNT_LOCK_DIR"
}

account_lock_path_matches_fd() {
    local mode uid gid path_identity fd_identity
    [[ "$ACCOUNT_LOCK_HELD" -eq 1 ]] || return 1
    account_lock_directory_is_secure || return 1
    [[ -f "$ACCOUNT_LOCK_FILE" && ! -L "$ACCOUNT_LOCK_FILE" ]] || return 1
    mode="$(stat -c '%a' -- "$ACCOUNT_LOCK_FILE")" || return 1
    uid="$(stat -c '%u' -- "$ACCOUNT_LOCK_FILE")" || return 1
    gid="$(stat -c '%g' -- "$ACCOUNT_LOCK_FILE")" || return 1
    [[ "$mode" == "600" && "$uid" == "0" && "$gid" == "0" ]] || return 1
    path_identity="$(stat -Lc '%d:%i' -- "$ACCOUNT_LOCK_FILE")" || return 1
    fd_identity="$(stat -Lc '%d:%i' -- "/proc/$$/fd/${ACCOUNT_LOCK_FD}")" || return 1
    [[ "$path_identity" == "$fd_identity" ]]
}

acquire_account_mutation_lock() {
    local mode uid gid
    prepare_account_lock_directory
    if [[ ! -e "$ACCOUNT_LOCK_FILE" ]]; then
        install -m 0600 -o root -g root -- /dev/null "$ACCOUNT_LOCK_FILE"
    fi
    [[ -f "$ACCOUNT_LOCK_FILE" && ! -L "$ACCOUNT_LOCK_FILE" ]] ||
        die "账号事务锁不是普通文件: $ACCOUNT_LOCK_FILE"
    mode="$(stat -c '%a' -- "$ACCOUNT_LOCK_FILE")"
    uid="$(stat -c '%u' -- "$ACCOUNT_LOCK_FILE")"
    gid="$(stat -c '%g' -- "$ACCOUNT_LOCK_FILE")"
    [[ "$mode" == "600" && "$uid" == "0" && "$gid" == "0" ]] ||
        die "账号事务锁必须为 root:root 0600"
    exec {ACCOUNT_LOCK_FD}<> "$ACCOUNT_LOCK_FILE"
    flock --exclusive --timeout 35 "$ACCOUNT_LOCK_FD" ||
        die "等待 ota-admin 在途账号事务完成超时"
    ACCOUNT_LOCK_HELD=1
    if ! account_lock_path_matches_fd; then
        flock --unlock "$ACCOUNT_LOCK_FD" || true
        exec {ACCOUNT_LOCK_FD}>&-
        ACCOUNT_LOCK_HELD=0
        die "账号事务锁路径与持锁 FD 不一致，拒绝 split-lock"
    fi
}

release_account_mutation_lock() {
    [[ "$ACCOUNT_LOCK_HELD" -eq 1 ]] || return 0
    flock --unlock "$ACCOUNT_LOCK_FD"
    exec {ACCOUNT_LOCK_FD}>&-
    ACCOUNT_LOCK_HELD=0
}

revalidate_account_lock_after_admin_stop() {
    account_lock_path_matches_fd && return 0
    log "ota-admin stop 改变了 RuntimeDirectory/锁 inode；服务已停止，正在安全重锁……"
    release_account_mutation_lock
    acquire_account_mutation_lock
    account_lock_path_matches_fd || die "ota-admin stop 后无法建立唯一账号事务锁"
}

[[ -f "$OTA_ADMIN_SOURCE" && ! -L "$OTA_ADMIN_SOURCE" ]] ||
    die "找不到可信的 ota_admin.py: $OTA_ADMIN_SOURCE"

validate_root_only_parent_chain() {
    local path="$1" label="$2" resolved current mode uid
    command -v readlink >/dev/null 2>&1 || die "缺少源码路径校验命令: readlink"
    [[ "$path" == /* ]] || die "$label 必须使用绝对规范路径"
    resolved="$(readlink -f -- "$path")" || die "无法解析 $label 路径"
    [[ "$resolved" == "$path" ]] || die "$label 路径不得包含符号链接或非规范分量"
    current="$(dirname -- "$resolved")"
    while :; do
        uid="$(stat -c '%u' -- "$current")"
        mode="$(stat -c '%a' -- "$current")"
        [[ "$uid" == "0" && $((8#$mode & 8#022)) -eq 0 ]] ||
            die "$label 父路径可被非 root 写: $current"
        [[ "$current" == "/" ]] && break
        current="$(dirname -- "$current")"
    done
}

validate_trusted_ota_admin_source() {
    local resolved mode uid
    validate_root_only_parent_chain "$OTA_ADMIN_SOURCE" "ota_admin.py 来源"
    resolved="$(readlink -f -- "$OTA_ADMIN_SOURCE")"
    [[ -f "$resolved" && ! -L "$OTA_ADMIN_SOURCE" ]] || die "ota_admin.py 来源不是普通文件"
    uid="$(stat -c '%u' -- "$resolved")"
    mode="$(stat -c '%a' -- "$resolved")"
    [[ "$uid" == "0" && $((8#$mode & 8#022)) -eq 0 ]] ||
        die "真实部署要求 ota_admin.py 为 root 拥有且不可由 group/other 写"
}

if [[ "$MODE" == "apply" && -z "$DEPLOY_ROOT" ]]; then
    validate_trusted_ota_admin_source
fi

validate_secret_file() {
    local path="$1" label="$2" min_length="$3"
    local check_mode="${4:-1}"
    local mode first extra=""
    [[ -f "$path" && ! -L "$path" ]] || die "$label 必须是普通文件（不能是符号链接）"
    mode="$(stat -c '%a' -- "$path" 2>/dev/null || true)"
    if [[ "$check_mode" -eq 1 ]]; then
        if [[ ! "$mode" =~ ^[46]00$ ]]; then
            if [[ -n "$DEPLOY_ROOT" && "${DEPLOY_OFFLINE_TEST:-0}" == "1" &&
                  "${DEPLOY_OFFLINE_TEST_RELAX_SECRET_MODE:-0}" == "1" ]]; then
                log "离线测试文件系统不支持 POSIX 权限；仅跳过秘密文件 mode 检查。"
            else
                die "$label 权限必须禁止 group/other 访问（建议 0600；当前 ${mode:-未知}）"
            fi
        fi
    fi
    IFS= read -r first < "$path" || true
    [[ ${#first} -ge $min_length ]] || die "$label 长度至少为 $min_length"
    [[ "$first" != *:* ]] || die "$label 不能包含冒号"
    if IFS= read -r extra < <(tail -n +2 -- "$path"); then
        [[ -z "$extra" ]] || die "$label 必须只有一行"
    fi
    unset first extra
}

if [[ -n "$DEVICEDATA_PASSWORD_FILE" ]]; then
    validate_secret_file "$DEVICEDATA_PASSWORD_FILE" "devicedata 密码文件" 8
fi
if [[ -n "$ADMIN_TOKEN_SOURCE" ]]; then
    validate_secret_file "$ADMIN_TOKEN_SOURCE" "管理令牌文件" 32
fi

if [[ "$MODE" == "apply" && -z "$DEPLOY_ROOT" ]]; then
    readonly REAL_STAGE_BASE="/var/tmp/no-teaching-online-services"
    if [[ ! -e "$REAL_STAGE_BASE" ]]; then
        install -d -m 0700 -o root -g root -- "$REAL_STAGE_BASE"
    fi
    [[ -d "$REAL_STAGE_BASE" && ! -L "$REAL_STAGE_BASE" &&
       "$(stat -c '%u:%g:%a' -- "$REAL_STAGE_BASE")" == "0:0:700" ]] ||
        die "真实部署 staging 根目录必须为 root:root 0700: $REAL_STAGE_BASE"
    STAGE_DIR="$(mktemp -d "${REAL_STAGE_BASE}/stage.XXXXXX")"
    chown root:root -- "$STAGE_DIR"
    chmod 0700 -- "$STAGE_DIR"
else
    STAGE_DIR="$(mktemp -d "${TMPDIR:-/tmp}/online-services-stage.XXXXXX")"
fi
command -v sha256sum >/dev/null 2>&1 || die "缺少源码哈希命令: sha256sum"
readonly OTA_ADMIN_STAGED="${STAGE_DIR}/ota_admin.py"
cp --no-dereference -- "$OTA_ADMIN_SOURCE" "$OTA_ADMIN_STAGED"
readonly OTA_ADMIN_STAGED_HASH="$(sha256sum -- "$OTA_ADMIN_STAGED" | awk '{print $1}')"
BACKUP_DIR=""
TRANSACTION_ACTIVE=0
TRANSACTION_COMMITTED=0
declare -a SNAPSHOT_TARGETS=()
declare -a SNAPSHOT_FILES=()
declare -a SNAPSHOT_EXISTED=()
declare -a DIR_PATHS=()
declare -a DIR_EXISTED=()
declare -a DIR_UIDS=()
declare -a DIR_GIDS=()
declare -a DIR_MODES=()
declare -a SERVICE_NAMES=()
declare -a SERVICE_ENABLED=()
declare -a SERVICE_ACTIVE=()
CREATED_FTP_GROUP=0
CREATED_DEVICEDATA=0
ADDED_DEVICEDATA_TO_FTPDATA=0
ACCOUNT_LOCK_FD=""
ACCOUNT_LOCK_HELD=0
ROLLBACK_INCOMPLETE=0

rollback_transaction() {
    local i path backup existed svc enabled active
    [[ "$TRANSACTION_ACTIVE" -eq 1 && "$TRANSACTION_COMMITTED" -eq 0 ]] || return 0
    set +e
    echo "部署失败，正在回滚本轮管理文件……" >&2

    # 回滚窗口内所有相关服务先保持停止，避免旧配置配上尚未撤销的账号/目录权限。
    if command -v systemctl >/dev/null 2>&1 && [[ -z "$DEPLOY_ROOT" ]]; then
        for ((i=${#SERVICE_NAMES[@]}-1; i>=0; i--)); do
            timeout --signal=TERM --kill-after=5s 20s \
                systemctl stop "${SERVICE_NAMES[$i]}" >/dev/null 2>&1 || ROLLBACK_INCOMPLETE=1
            systemctl is-active --quiet "${SERVICE_NAMES[$i]}" && ROLLBACK_INCOMPLETE=1
        done
    fi

    for ((i=${#SNAPSHOT_TARGETS[@]}-1; i>=0; i--)); do
        path="${SNAPSHOT_TARGETS[$i]}"
        backup="${SNAPSHOT_FILES[$i]}"
        existed="${SNAPSHOT_EXISTED[$i]}"
        rm -f -- "$path" || ROLLBACK_INCOMPLETE=1
        if [[ "$existed" == "1" ]]; then
            mkdir -p -- "$(dirname -- "$path")" || ROLLBACK_INCOMPLETE=1
            cp -a -- "$backup" "$path" || ROLLBACK_INCOMPLETE=1
        fi
    done

    if [[ -z "$DEPLOY_ROOT" ]]; then
        if [[ "$ADDED_DEVICEDATA_TO_FTPDATA" -eq 1 ]]; then
            timeout --signal=TERM --kill-after=5s 30s \
                gpasswd -d devicedata ftpdata >/dev/null 2>&1 || true
            if user_in_ftpdata devicedata; then
                echo "警告: 回滚未能撤销 devicedata 的 ftpdata 组成员关系" >&2
                ROLLBACK_INCOMPLETE=1
            fi
        fi
        if [[ "$CREATED_DEVICEDATA" -eq 1 ]]; then
            timeout --signal=TERM --kill-after=5s 20s userdel devicedata >/dev/null 2>&1 || true
            id devicedata >/dev/null 2>&1 && ROLLBACK_INCOMPLETE=1
            if lookup_group_entry devicedata >/dev/null 2>&1; then
                timeout --signal=TERM --kill-after=5s 20s groupdel devicedata >/dev/null 2>&1 || true
                lookup_group_entry devicedata >/dev/null 2>&1 && ROLLBACK_INCOMPLETE=1
            fi
        fi
        if [[ "$CREATED_FTP_GROUP" -eq 1 ]]; then
            timeout --signal=TERM --kill-after=5s 20s groupdel ftpdata >/dev/null 2>&1 || true
            lookup_group_entry ftpdata >/dev/null 2>&1 && ROLLBACK_INCOMPLETE=1
        fi
    fi

    for ((i=${#DIR_PATHS[@]}-1; i>=0; i--)); do
        path="${DIR_PATHS[$i]}"
        if [[ "${DIR_EXISTED[$i]}" == "1" ]]; then
            if [[ -z "$DEPLOY_ROOT" ]]; then
                chown "${DIR_UIDS[$i]}:${DIR_GIDS[$i]}" -- "$path" >/dev/null 2>&1 ||
                    ROLLBACK_INCOMPLETE=1
            fi
            chmod "${DIR_MODES[$i]}" -- "$path" >/dev/null 2>&1 || ROLLBACK_INCOMPLETE=1
        else
            rmdir -- "$path" >/dev/null 2>&1 || ROLLBACK_INCOMPLETE=1
        fi
    done

    # 文件、账号、组和目录元数据全部恢复后，最后才恢复原 service 状态。
    if command -v systemctl >/dev/null 2>&1 && [[ -z "$DEPLOY_ROOT" ]]; then
        timeout --signal=TERM --kill-after=5s 20s systemctl daemon-reload >/dev/null 2>&1 ||
            ROLLBACK_INCOMPLETE=1
        for ((i=${#SERVICE_NAMES[@]}-1; i>=0; i--)); do
            svc="${SERVICE_NAMES[$i]}"
            enabled="${SERVICE_ENABLED[$i]}"
            active="${SERVICE_ACTIVE[$i]}"
            case "$enabled" in
                enabled)
                    timeout --signal=TERM --kill-after=5s 20s \
                        systemctl enable "$svc" >/dev/null 2>&1 || ROLLBACK_INCOMPLETE=1
                    [[ "$(systemctl is-enabled "$svc" 2>/dev/null || true)" == "enabled" ]] ||
                        ROLLBACK_INCOMPLETE=1
                    ;;
                enabled-runtime)
                    # 本轮 systemctl enable --now 可能已留下持久链接；先清除，再只恢复
                    # /run 范围的 runtime enable，避免跨重启状态漂移。
                    timeout --signal=TERM --kill-after=5s 20s \
                        systemctl disable "$svc" >/dev/null 2>&1 || ROLLBACK_INCOMPLETE=1
                    timeout --signal=TERM --kill-after=5s 20s \
                        systemctl enable --runtime "$svc" >/dev/null 2>&1 || ROLLBACK_INCOMPLETE=1
                    [[ "$(systemctl is-enabled "$svc" 2>/dev/null || true)" == "enabled-runtime" ]] ||
                        ROLLBACK_INCOMPLETE=1
                    ;;
                *)
                    timeout --signal=TERM --kill-after=5s 20s \
                        systemctl disable "$svc" >/dev/null 2>&1 || ROLLBACK_INCOMPLETE=1
                    systemctl is-enabled --quiet "$svc" && ROLLBACK_INCOMPLETE=1
                    ;;
            esac
            if [[ "$active" == "active" ]]; then
                timeout --signal=TERM --kill-after=5s 20s systemctl start "$svc" >/dev/null 2>&1 ||
                    ROLLBACK_INCOMPLETE=1
                if ! systemctl is-active --quiet "$svc"; then
                    echo "警告: 回滚后服务未恢复 active: $svc" >&2
                    ROLLBACK_INCOMPLETE=1
                fi
            else
                timeout --signal=TERM --kill-after=5s 20s systemctl stop "$svc" >/dev/null 2>&1 ||
                    ROLLBACK_INCOMPLETE=1
                systemctl is-active --quiet "$svc" && ROLLBACK_INCOMPLETE=1
            fi
        done
    fi
    if [[ "$ROLLBACK_INCOMPLETE" -eq 0 ]]; then
        echo "回滚完成；备份保留在 ${BACKUP_DIR:-未创建}。" >&2
    else
        echo "严重: 回滚未完整完成，服务保持状态需人工核验；备份在 ${BACKUP_DIR:-未创建}。" >&2
    fi
}

on_exit() {
    local rc=$?
    if [[ $rc -ne 0 ]]; then
        rollback_transaction
    fi
    rm -rf -- "$STAGE_DIR"
    exit "$rc"
}
trap on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

cat > "${STAGE_DIR}/nginx-ota.conf" <<'NGINX'
server {
    listen 8090;
    server_name _;
    root /var/www;
    autoindex off;

    location ~ ^/ota/(neutral|brand)/latest(?:-v3)?\.json$ {
        limit_except GET { deny all; }
        try_files $uri =404;
        default_type application/json;
        add_header Cache-Control "no-cache, no-store, must-revalidate" always;
        add_header X-Content-Type-Options "nosniff" always;
    }

    location ~ ^/ota/(neutral|brand)/.*\.(exe|zip)$ {
        limit_except GET { deny all; }
        try_files $uri =404;
        add_header X-Content-Type-Options "nosniff" always;
    }

    # ota_admin.py 只监听回环地址；8090 是唯一外部入口。
    location ^~ /admin/ {
        # 安全默认：管理令牌和改密 JSON 不得在明文公网传输。远程管理只能经
        # SSH/TLS 可信隧道落到服务器 loopback，再访问本 location。
        allow 127.0.0.1;
        allow ::1;
        deny all;
        limit_except GET POST PATCH DELETE { deny all; }
        proxy_pass http://127.0.0.1:8091;
        proxy_http_version 1.1;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        proxy_set_header X-Admin-Token $http_x_admin_token;
        proxy_connect_timeout 3s;
        # 超时层级必须保持：ota-admin 子命令 10s < nginx 15s < Qt 请求 20s。
        proxy_read_timeout 15s;
        client_max_body_size 64k;
    }

    location / {
        return 404;
    }
}
NGINX

cat > "${STAGE_DIR}/ota-admin.service" <<'UNIT'
[Unit]
Description=NoTeaching-Robot OTA administration API
After=network.target
Wants=network.target

[Service]
Type=simple
User=root
Group=root
RuntimeDirectory=no-teaching-ota
RuntimeDirectoryMode=0700
RuntimeDirectoryPreserve=yes
WorkingDirectory=/opt/ota-admin
ExecStart=/usr/bin/python3 -I /opt/ota-admin/ota_admin.py
Restart=on-failure
RestartSec=3s
UMask=0077
NoNewPrivileges=true
PrivateTmp=true
ProtectHome=true
ProtectKernelTunables=true
ProtectKernelModules=true
ProtectControlGroups=true
RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6

[Install]
WantedBy=multi-user.target
UNIT

cat > "${STAGE_DIR}/clean-devicedata" <<'CRON'
#!/bin/sh
set -eu
DATA_ROOT="${NO_TEACHING_DATA_ROOT:-/srv/devicedata/data}"
case "$DATA_ROOT" in
    /*) ;;
    *) echo "拒绝非绝对 DATA_ROOT" >&2; exit 1 ;;
esac
[ -d "$DATA_ROOT" ] && [ ! -L "$DATA_ROOT" ] || exit 0

# 新客户端文件名末尾编码预期字节数。静置超过 10 分钟后仍不匹配即视为失败残件；
# 完整匹配文件保留，旧命名仍走 30 天策略。仅处理 regular file，不解压/重命名。
python3 -I - "$DATA_ROOT" <<'PY'
import os
import re
import stat
import sys
import time

root = os.path.abspath(sys.argv[1])
pattern = re.compile(
    r"^.+_[0-9]{8}T[0-9]{9}_[0-9A-Fa-f]{12}_([0-9]{1,20})\.zip$"
)
cutoff = time.time() - 10 * 60

for directory, dirnames, filenames in os.walk(root, topdown=True, followlinks=False):
    dirnames[:] = [
        name for name in dirnames
        if not os.path.islink(os.path.join(directory, name))
    ]
    directory_fd = None
    if os.name == "posix":
        directory_fd = os.open(directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        for name in filenames:
            match = pattern.fullmatch(name)
            if match is None:
                continue
            path = os.path.join(directory, name)
            try:
                before = (
                    os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
                    if directory_fd is not None else os.stat(path, follow_symlinks=False)
                )
            except FileNotFoundError:
                continue
            if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or before.st_mtime > cutoff:
                continue
            if before.st_size == int(match.group(1)):
                continue
            try:
                current = (
                    os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
                    if directory_fd is not None else os.stat(path, follow_symlinks=False)
                )
            except FileNotFoundError:
                continue
            identity = ("st_dev", "st_ino", "st_size", "st_mtime_ns")
            if not stat.S_ISREG(current.st_mode) or any(
                getattr(current, field) != getattr(before, field) for field in identity
            ):
                continue
            if directory_fd is not None:
                os.unlink(name, dir_fd=directory_fd)
            else:
                os.unlink(path)
    finally:
        if directory_fd is not None:
            os.close(directory_fd)
PY

find "$DATA_ROOT" -xdev -type f -links 1 -mtime +30 -delete
# depth=1 是账号授权根目录；即使暂时为空也必须保留，否则次日登录会失效。
find "$DATA_ROOT" -xdev -mindepth 2 -type d -empty -delete
CRON

build_vsftpd_candidate() {
    local source="$1" output="$2"
    [[ -f "$source" ]] || source="/dev/null"
    awk -v configured_pasv="$PASV_ADDRESS" '
        function fail(reason) {
            print "错误: vsftpd 既有配置不安全: " reason > "/dev/stderr"
            failed = 1
            exit 42
        }
        function safe_boolean_extra(key) {
            return key == "dirmessage_enable" || key == "use_localtime" || \
                   key == "connect_from_port_20" || key == "log_ftp_protocol" || \
                   key == "syslog_enable" || key == "dual_log_enable" || \
                   key == "xferlog_std_format"
        }
        BEGIN {
            n = 20
            keys[1]="listen";                  wanted["listen"]="YES"
            keys[2]="listen_ipv6";             wanted["listen_ipv6"]="NO"
            keys[3]="anonymous_enable";        wanted["anonymous_enable"]="NO"
            keys[4]="local_enable";            wanted["local_enable"]="YES"
            keys[5]="write_enable";            wanted["write_enable"]="YES"
            keys[6]="local_umask";              wanted["local_umask"]="002"
            keys[7]="chroot_local_user";        wanted["chroot_local_user"]="YES"
            keys[8]="allow_writeable_chroot";  wanted["allow_writeable_chroot"]="NO"
            keys[9]="local_root";               wanted["local_root"]="/srv/devicedata"
            keys[10]="pasv_enable";             wanted["pasv_enable"]="YES"
            keys[11]="pasv_min_port";           wanted["pasv_min_port"]="40000"
            keys[12]="pasv_max_port";           wanted["pasv_max_port"]="40100"
            keys[13]="utf8_filesystem";         wanted["utf8_filesystem"]="YES"
            keys[14]="xferlog_enable";          wanted["xferlog_enable"]="YES"
            keys[15]="xferlog_file";            wanted["xferlog_file"]="/var/log/vsftpd.log"
            keys[16]="idle_session_timeout";    wanted["idle_session_timeout"]="300"
            keys[17]="data_connection_timeout"; wanted["data_connection_timeout"]="120"
            keys[18]="userlist_enable";         wanted["userlist_enable"]="YES"
            keys[19]="userlist_file";           wanted["userlist_file"]="/etc/vsftpd.userlist"
            keys[20]="userlist_deny";           wanted["userlist_deny"]="NO"
            n++
            keys[n]="user_config_dir";          wanted["user_config_dir"]="/etc/vsftpd_user_conf"
            n++
            keys[n]="listen_port";              wanted["listen_port"]="21"
            n++
            keys[n]="guest_enable";             wanted["guest_enable"]="NO"
            n++
            keys[n]="pam_service_name";         wanted["pam_service_name"]="vsftpd"
            n++
            keys[n]="secure_chroot_dir";        wanted["secure_chroot_dir"]="/var/run/vsftpd/empty"
            n++
            keys[n]="ssl_enable";               wanted["ssl_enable"]="NO"
            n++
            keys[n]="pasv_promiscuous";         wanted["pasv_promiscuous"]="NO"
            n++
            keys[n]="port_promiscuous";         wanted["port_promiscuous"]="NO"
            n++
            keys[n]="download_enable";          wanted["download_enable"]="YES"
            n++
            keys[n]="chmod_enable";             wanted["chmod_enable"]="YES"
            n++
            keys[n]="file_open_mode";           wanted["file_open_mode"]="0644"
            n++
            keys[n]="delete_failed_uploads";    wanted["delete_failed_uploads"]="YES"
            strict["anonymous_enable"] = 1
            strict["guest_enable"] = 1
            strict["listen_port"] = 1
            strict["pam_service_name"] = 1
            strict["secure_chroot_dir"] = 1
            strict["ssl_enable"] = 1
            strict["pasv_promiscuous"] = 1
            strict["port_promiscuous"] = 1
            strict["local_umask"] = 1
            strict["download_enable"] = 1
            strict["chmod_enable"] = 1
            strict["file_open_mode"] = 1
            strict["userlist_enable"] = 1
            strict["userlist_file"] = 1
            strict["userlist_deny"] = 1
            if (configured_pasv != "") {
                n++
                keys[n]="pasv_address";         wanted["pasv_address"]=configured_pasv
                n++
                keys[n]="pasv_addr_resolve";    wanted["pasv_addr_resolve"]="YES"
            }
        }
        {
            raw = $0
            line = $0
            gsub(/^[ \t]+|[ \t]+$/, "", line)
            if (line == "" || line ~ /^#/) {
                print raw
                next
            }
            if (index(line, "=") == 0) fail("存在无效行")
            key = line
            sub(/[ \t]*=.*/, "", key)
            value = line
            sub(/^[^=]*=[ \t]*/, "", value)
            sub(/[ \t]+$/, "", value)
            if (key !~ /^[a-z][a-z0-9_]*$/) fail("无效指令名 " key)
            if (key in seen) fail("重复指令 " key)
            seen[key] = 1

            if (key in wanted) {
                if ((key in strict) && value != wanted[key]) {
                    fail(key " 必须为 " wanted[key])
                }
                print key "=" wanted[key]
                next
            }
            if (safe_boolean_extra(key) && (value == "YES" || value == "NO")) {
                print key "=" value
                next
            }
            if (key == "hide_ids" && value == "YES") {
                print "hide_ids=YES"
                next
            }
            if (key == "rsa_cert_file" && value == "/etc/ssl/certs/ssl-cert-snakeoil.pem") {
                print key "=" value
                next
            }
            if (key == "rsa_private_key_file" && value == "/etc/ssl/private/ssl-cert-snakeoil.key") {
                print key "=" value
                next
            }
            if (configured_pasv == "" && key == "pasv_address" &&
                value ~ /^[A-Za-z0-9][A-Za-z0-9.-]*[A-Za-z0-9]$/ && value !~ /\.\./) {
                print key "=" value
                next
            }
            if (configured_pasv == "" && key == "pasv_addr_resolve" &&
                (value == "YES" || value == "NO")) {
                print key "=" value
                next
            }
            if (key ~ /_enable$/ && value == "NO") {
                print key "=NO"
                next
            }
            fail("不支持可能放权或漂移的指令 " key)
        }
        END {
            if (failed) exit 42
            for (i = 1; i <= n; i++) {
                key = keys[i]
                if (!(key in seen)) print key "=" wanted[key]
            }
        }
    ' "$source" > "$output" || die "vsftpd 全局配置校验失败"
}

remove_retired_uploader_candidate() {
    local source="$1" output="$2"
    [[ -f "$source" ]] || source="/dev/null"
    awk '
        {
            logical = $0
            gsub(/^[ \t]+|[ \t]+$/, "", logical)
            if (logical == "uploader") next
            print
        }
    ' "$source" > "$output"
}

validate_upload_only_user_conf() {
    local account="$1" path="${VSFTPD_USER_CONF_DIR}/$1"
    [[ -f "$path" && ! -L "$path" ]] ||
        die "账号 ${account} 的按用户配置不是普通文件"
    [[ "$(stat -c '%s' -- "$path")" -le 65536 ]] ||
        die "账号 ${account} 的按用户配置超过 64KiB"
    awk -v account="$account" -v expected_root="/srv/devicedata" '
        function fail(reason) {
            print "错误: 账号 " account " 的 upload-only 配置不安全: " reason > "/dev/stderr"
            failed = 1
            exit 42
        }
        {
            line = $0
            gsub(/^[ \t]+|[ \t]+$/, "", line)
            if (line == "" || line ~ /^#/) next
            if (index(line, "=") == 0) fail("无效行")
            key = line
            sub(/[ \t]*=.*/, "", key)
            value = line
            sub(/^[^=]*=[ \t]*/, "", value)
            sub(/[ \t]+$/, "", value)
            if (key !~ /^[a-z][a-z0-9_]*$/ || key in seen) fail("重复或无效指令 " key)
            seen[key] = 1
            if (key == "download_enable" || key == "chmod_enable") {
                if (value != "NO") fail(key " 必须为 NO")
                next
            }
            if (key == "file_open_mode") {
                if (value != "0440") fail("file_open_mode 必须为 0440")
                next
            }
            if (key == "local_umask") {
                if (value != "007") fail("local_umask 必须为 007")
                next
            }
            if (key == "local_root") {
                if (value != expected_root) fail("local_root 必须保持 root-owned /srv/devicedata chroot")
                next
            }
            if (key == "cmds_denied") {
                count = split(value, commands, ",")
                for (i = 1; i <= count; i++) {
                    command = commands[i]
                    if (command !~ /^[A-Z][A-Z0-9_]*$/ || command in denied) {
                        fail("cmds_denied 格式或重复项无效")
                    }
                    denied[command] = 1
                }
                next
            }
            if (key == "hide_ids" && value == "YES") next
            if (key ~ /_enable$/ && value == "NO") next
            fail("不支持指令 " key)
        }
        END {
            if (failed) exit 42
            if (!("download_enable" in seen) || !("chmod_enable" in seen) ||
                !("file_open_mode" in seen) || !("local_umask" in seen) ||
                !("local_root" in seen) || !("cmds_denied" in seen)) exit 43
            split("DELE RMD RNFR RNTO APPE REST", required, " ")
            for (i = 1; i <= 6; i++) if (!(required[i] in denied)) exit 44
        }
    ' "$path" || die "账号 ${account} 的 upload-only 配置不满足 canonical"
}

validate_device_directory() {
    local account="$1" path="${FTP_DATA}/$1" account_entry admin_entry account_uid admin_gid attrs mode uid gid unsafe
    [[ "$account" =~ ^[a-z][a-z0-9_-]{2,31}$ && "$account" != "uploader" ]] ||
        die "设备账号名无效或仍使用已退役 uploader"
    [[ -d "$path" && ! -L "$path" ]] ||
        die "账号 ${account} 缺少真实独立目录 ${path}；拒绝共享目录上传"
    unsafe="$(find "$path" -xdev \( -type l -o \( -type f -links +1 \) \) -print -quit)"
    [[ -z "$unsafe" ]] ||
        die "账号 ${account} 目录含符号链接/硬链接，无法证明设备边界"
    if [[ -z "$DEPLOY_ROOT" ]]; then
        account_entry="$(lookup_passwd_entry "$account")"
        admin_entry="$(lookup_passwd_entry devicedata)"
        IFS=: read -r _ _ account_uid _ _ _ _ _ <<< "$account_entry"
        IFS=: read -r _ _ _ admin_gid _ _ _ _ <<< "$admin_entry"
        attrs="$(stat -c '%a:%u:%g' -- "$path")"
        IFS=: read -r mode uid gid <<< "$attrs"
        [[ "$mode" == "2770" && "$uid" == "$account_uid" && "$gid" == "$admin_gid" ]] ||
            die "账号 ${account} 目录必须为 account:devicedata-private-gid 2770"
    fi
}

append_line_candidate() {
    local source="$1" output="$2" wanted="$3"
    [[ -f "$source" ]] || source="/dev/null"
    awk -v wanted="$wanted" '
        { print; if ($0 == wanted) found = 1 }
        END { if (!found) print wanted }
    ' "$source" > "$output"
}

verify_userlist_preserved() {
    local original="$1" candidate="$2" line
    [[ -f "$original" ]] || return 0
    while IFS= read -r line || [[ -n "$line" ]]; do
        [[ -z "$line" || "$line" == \#* ]] && continue
        [[ "$line" == "uploader" ]] && continue
        grep -Fqx -- "$line" "$candidate" || die "内部错误: FTP 白名单候选丢失既有账号"
    done < "$original"
}

refresh_ftp_candidates() {
    if [[ -f "$VSFTPD_USERLIST" ]]; then
        cp -- "$VSFTPD_USERLIST" "${STAGE_DIR}/original-vsftpd.userlist"
    else
        : > "${STAGE_DIR}/original-vsftpd.userlist"
    fi
    build_vsftpd_candidate "$VSFTPD_CONFIG" "${STAGE_DIR}/vsftpd.conf"
    remove_retired_uploader_candidate "$VSFTPD_USERLIST" \
        "${STAGE_DIR}/vsftpd.userlist.without-uploader"
    append_line_candidate "${STAGE_DIR}/vsftpd.userlist.without-uploader" \
        "${STAGE_DIR}/vsftpd.userlist" "devicedata"
    append_line_candidate "$SHELLS_FILE" "${STAGE_DIR}/shells" "/usr/sbin/nologin"
    verify_userlist_preserved "$VSFTPD_USERLIST" "${STAGE_DIR}/vsftpd.userlist"
}

validate_ftp_candidates() {
    local pasv_count denied_command
    sh -n "${STAGE_DIR}/clean-devicedata"
    grep -Fqx 'user_config_dir=/etc/vsftpd_user_conf' "${STAGE_DIR}/vsftpd.conf" ||
        die "内部错误: vsftpd 缺少按用户权限配置目录"
    grep -Fqx 'local_umask=002' "${STAGE_DIR}/vsftpd.conf" ||
        die "内部错误: vsftpd 共享组 umask 漂移"
    grep -Fqx 'delete_failed_uploads=YES' "${STAGE_DIR}/vsftpd.conf" ||
        die "内部错误: 失败上传清理门禁漂移"
    pasv_count="$(grep -Ec '^[[:space:]]*pasv_address[[:space:]]*=' "${STAGE_DIR}/vsftpd.conf" || true)"
    [[ "$pasv_count" -le 1 ]] || die "vsftpd 存在重复 pasv_address，拒绝猜测公网回址"
    if [[ -n "$PASV_ADDRESS" ]]; then
        grep -Fqx "pasv_address=${PASV_ADDRESS}" "${STAGE_DIR}/vsftpd.conf" ||
            die "内部错误: pasv_address 未写入候选配置"
        grep -Fqx 'pasv_addr_resolve=YES' "${STAGE_DIR}/vsftpd.conf" ||
            die "内部错误: pasv_address 主机解析未启用"
        PASV_STATUS="显式设置为 ${PASV_ADDRESS}"
    elif [[ "$pasv_count" -eq 1 ]]; then
        PASV_STATUS="保留既有 pasv_address（本轮未猜测）"
    else
        PASV_STATUS="未配置；NAT/公网服务器的被动连接可能失败，请提供 --pasv-address"
    fi
    grep -Fqx 'devicedata' "${STAGE_DIR}/vsftpd.userlist" ||
        die "内部错误: FTP 白名单缺少 devicedata"
    ! grep -Fqx 'uploader' "${STAGE_DIR}/vsftpd.userlist" ||
        die "内部错误: 共享 uploader 未从候选白名单退役"
}

validate_final_ftp_account_state() {
    local account conf
    local -A seen_accounts=()
    while IFS= read -r account || [[ -n "$account" ]]; do
        [[ "$account" =~ ^[a-z][a-z0-9_-]{2,31}$ ]] ||
            die "vsftpd.userlist 含无效账号名: $account"
        [[ "$account" != "uploader" ]] ||
            die "共享 uploader 已退役，禁止出现在最终 FTP 白名单"
        [[ ! -v "seen_accounts[$account]" ]] ||
            die "vsftpd.userlist 含重复账号: $account"
        seen_accounts["$account"]=1
        validate_existing_ftp_account "$account"
        user_in_ftpdata "$account" ||
            die "白名单账号 ${account} 不是 ftpdata 成员"
        conf="${VSFTPD_USER_CONF_DIR}/$account"
        if [[ "$account" == "devicedata" ]]; then
            [[ ! -e "$conf" && ! -L "$conf" ]] ||
                die "devicedata 必须为 full，不能有按用户限制文件"
        else
            validate_device_private_group "$account"
            validate_upload_only_user_conf "$account"
            validate_device_directory "$account"
        fi
    done < <(awk '
        { gsub(/^[ \t]+|[ \t]+$/, "") }
        NF && $0 !~ /^#/ { print }
    ' "$VSFTPD_USERLIST")
    [[ -v 'seen_accounts[devicedata]' ]] || die "最终白名单缺少 devicedata 管理账号"
    validate_devicedata_private_group
}

validate_preexisting_device_isolation() {
    local account
    [[ -z "$DEPLOY_ROOT" || -n "$DEPLOY_NSS_FIXTURE_DIR" ]] || return 0
    if account_exists devicedata; then
        validate_existing_ftp_account devicedata
        validate_devicedata_private_group
    fi
    while IFS= read -r account || [[ -n "$account" ]]; do
        [[ -z "$account" || "$account" == \#* || "$account" == "devicedata" ]] && continue
        validate_existing_ftp_account "$account"
        user_in_ftpdata "$account" || die "设备账号 ${account} 不是 ftpdata 成员"
        validate_device_private_group "$account"
        validate_upload_only_user_conf "$account"
        validate_device_directory "$account"
    done < <(awk '{ gsub(/^[ \t]+|[ \t]+$/, ""); if (NF && $0 !~ /^#/) print }' \
        "${STAGE_DIR}/vsftpd.userlist")
}

refresh_ftp_candidates
validate_ftp_candidates
validate_preexisting_device_isolation

if command -v python3 >/dev/null 2>&1; then
    OTA_ADMIN_STAGED_FOR_CHECK="$OTA_ADMIN_STAGED" python3 -I - <<'PY'
import os
from pathlib import Path
path = Path(os.environ["OTA_ADMIN_STAGED_FOR_CHECK"])
compile(path.read_bytes(), str(path), "exec")
PY
else
    die "内部错误: 预装依赖校验通过后 python3 不可用"
fi

existing_users=0
if [[ -f "$VSFTPD_USERLIST" ]]; then
    existing_users="$(awk 'NF && $0 !~ /^#/ {n++} END {print n+0}' "$VSFTPD_USERLIST")"
fi

if [[ -n "$DEPLOY_UFW_RULES_FIXTURE" ]]; then
    validate_ufw_rules_file "$DEPLOY_UFW_RULES_FIXTURE"
fi

if [[ "$MODE" == "dry-run" ]]; then
    cat <<EOF
=== 在线服务部署预演（未修改任何文件）===
将验证预装依赖并配置: nginx、vsftpd、python3、ota-admin.service（绝不运行 apt）
将原子管理: ${NGINX_SITE}
             ${VSFTPD_CONFIG}
             ${OTA_ADMIN_UNIT}
将保留除已退役 uploader 外的既有 FTP 白名单，并在缺少时追加 devicedata。
账号能力: devicedata=唯一 full；普通账号=write-once + /data/<account> 独立目录。
PASV 公网回址: ${PASV_STATUS}
不会删除其他 nginx 站点，不会重置既有 FTP 密码，不会递归改写上传数据。
UFW 规则: --apply 强制 active，FTP 仅允许 ${FTP_ALLOW_CIDR:-<必须提供的私网/VPN CIDR>}
容量边界: ${CAPACITY_STATUS}

确认计划后，使用 --apply；无人值守执行还必须显式给出 --yes。
EOF
    exit 0
fi

log "PASV 公网回址: ${PASV_STATUS}"

if [[ -n "$DEPLOY_ROOT" ]]; then
    [[ "${DEPLOY_OFFLINE_TEST:-0}" == "1" ]] || die "离线根目录保护失效"
else
    [[ -r /etc/os-release ]] || die "无法识别操作系统"
    # shellcheck disable=SC1091
    . /etc/os-release
    [[ "${ID:-}" == "ubuntu" ]] || die "仅支持 Ubuntu 22.04/24.04（当前 ID=${ID:-未知}）"
    case "${VERSION_ID:-}" in
        22.04|24.04) ;;
        *) die "仅支持 Ubuntu 22.04/24.04（当前 VERSION_ID=${VERSION_ID:-未知}）" ;;
    esac
fi

if [[ -z "$DEPLOY_ROOT" ]]; then
    for secret_source in "$DEVICEDATA_PASSWORD_FILE" "$ADMIN_TOKEN_SOURCE"; do
        [[ -z "$secret_source" ]] && continue
        [[ "$(stat -c '%u' -- "$secret_source")" == "0" ]] ||
            die "--apply 使用的秘密文件必须由 root 拥有: $secret_source"
        validate_root_only_parent_chain "$secret_source" "秘密文件"
    done
fi

if [[ "$ASSUME_YES" -ne 1 ]]; then
    [[ -t 0 ]] || die "非交互执行必须显式给出 --yes"
    cat <<EOF
即将在本机安装配置并重载 nginx/vsftpd/ota-admin。
现有 FTP 白名单条目数: ${existing_users}（全部保留）
不会删除其他站点或上传数据。继续请输入 APPLY:
EOF
    IFS= read -r confirmation
    [[ "$confirmation" == "APPLY" ]] || die "用户取消"
    unset confirmation
fi

if account_exists devicedata; then
    [[ -z "$DEVICEDATA_PASSWORD_FILE" ]] ||
        die "既有 devicedata 禁止由部署器改密；请通过 ota-admin 单账号轮换"
else
    [[ -n "$DEVICEDATA_PASSWORD_FILE" ]] ||
        die "新建 devicedata 必须提供 --devicedata-password-file；脚本拒绝生成无法取回的 FTP 密码"
fi
if [[ -z "$DEPLOY_ROOT" ]]; then
    if getent group ftpdata >/dev/null 2>&1; then
        validate_ftpdata_group
    fi
    account_exists devicedata && validate_existing_ftp_account devicedata
fi

readonly BACKUP_BASE="$(target_path /var/backups/no-teaching-online-services)"
if [[ -n "$DEPLOY_ROOT" ]]; then
    mkdir -p -- "$BACKUP_BASE"
    if [[ "${DEPLOY_OFFLINE_TEST_RELAX_SECRET_MODE:-0}" != "1" ]]; then
        chmod 0700 -- "$BACKUP_BASE"
    fi
else
    install -d -m 0700 -o root -g root -- "$BACKUP_BASE"
fi
BACKUP_DIR="${BACKUP_BASE}/$(date -u +%Y%m%dT%H%M%SZ)-$$"
if [[ -n "$DEPLOY_ROOT" ]]; then
    mkdir -p -- "$BACKUP_DIR"
    if [[ "${DEPLOY_OFFLINE_TEST_RELAX_SECRET_MODE:-0}" != "1" ]]; then
        chmod 0700 -- "$BACKUP_DIR"
    fi
else
    install -d -m 0700 -o root -g root -- "$BACKUP_DIR"
fi
TRANSACTION_ACTIVE=1

record_service_state() {
    local svc="$1"
    SERVICE_NAMES+=("$svc")
    SERVICE_ENABLED+=("$(systemctl is-enabled "$svc" 2>/dev/null || true)")
    SERVICE_ACTIVE+=("$(systemctl is-active "$svc" 2>/dev/null || true)")
}

if [[ -z "$DEPLOY_ROOT" ]]; then
    if [[ "$CONFIGURE_UFW" -eq 1 ]]; then
        audit_existing_ufw_ftp_rules
    fi
    if [[ "$ENABLE_UFW" -eq 1 ]]; then
        validate_ssh_listener_before_ufw_enable
    fi
fi

if [[ -z "$DEPLOY_ROOT" ]]; then
    record_service_state nginx
    record_service_state vsftpd
    record_service_state ota-admin
    if systemctl is-active --quiet ota-admin &&
       { [[ ! -f "$OTA_ADMIN_TARGET" ]] ||
         ! grep -Fq "$ACCOUNT_LOCK_FILE" "$OTA_ADMIN_TARGET"; }; then
        die "运行中的 ota-admin 尚不支持共享账号事务锁；请先在维护窗口排空并停止旧服务"
    fi
fi

if [[ -z "$DEPLOY_ROOT" ]]; then
    command -v flock >/dev/null 2>&1 || die "缺少账号事务锁命令: flock"
    acquire_account_mutation_lock
    if systemctl is-active --quiet ota-admin; then
        log "暂停 ota-admin，消除部署期间 FTP 白名单并发写入……"
        systemctl stop ota-admin
        if systemctl is-active --quiet ota-admin; then
            die "ota-admin 未能停止，拒绝覆盖 FTP 白名单"
        fi
    fi
    # 兼容首次升级的旧 unit：若 stop 删除 RuntimeDirectory/锁路径，当前 FD 仍锁着
    # 已 unlink 的旧 inode。服务 inactive 后必须关闭旧 FD 并在安全目录重新 LOCK_EX。
    revalidate_account_lock_after_admin_stop
    if systemctl is-active --quiet vsftpd; then
        log "暂停 vsftpd，冻结上传会话后再审计 write-once 存量状态……"
        systemctl stop vsftpd
        if systemctl is-active --quiet vsftpd; then
            die "vsftpd 未能停止，拒绝在活动上传期间修改权限或审计存量文件"
        fi
    fi
    # 依赖已由外部维护窗口预装；停服后重读，启动前一直保持停止。
    refresh_ftp_candidates
    validate_ftp_candidates
fi

OTA_ADMIN_STAGED_FOR_CHECK="$OTA_ADMIN_STAGED" python3 -I - <<'PY'
import os
from pathlib import Path
path = Path(os.environ["OTA_ADMIN_STAGED_FOR_CHECK"])
compile(path.read_bytes(), str(path), "exec")
PY

snapshot_target() {
    local target="$1" i backup
    for ((i=0; i<${#SNAPSHOT_TARGETS[@]}; i++)); do
        [[ "${SNAPSHOT_TARGETS[$i]}" == "$target" ]] && return 0
    done
    backup="${BACKUP_DIR}/file-${#SNAPSHOT_TARGETS[@]}"
    SNAPSHOT_TARGETS+=("$target")
    SNAPSHOT_FILES+=("$backup")
    if [[ -e "$target" || -L "$target" ]]; then
        cp -a -- "$target" "$backup"
        SNAPSHOT_EXISTED+=("1")
    else
        SNAPSHOT_EXISTED+=("0")
    fi
    printf '%q\t%s\n' "$target" "${SNAPSHOT_EXISTED[-1]}" >> "${BACKUP_DIR}/manifest.tsv"
}

atomic_install_file() {
    local source="$1" target="$2" mode="$3" owner="$4" group="$5"
    local dir base temp
    dir="$(dirname -- "$target")"
    base="$(basename -- "$target")"
    mkdir -p -- "$dir"
    temp="$(mktemp "${dir}/.${base}.tmp.XXXXXX")"
    if [[ -n "$DEPLOY_ROOT" ]]; then
        cp -- "$source" "$temp"
        if [[ "${DEPLOY_OFFLINE_TEST_RELAX_SECRET_MODE:-0}" != "1" ]]; then
            chmod "$mode" -- "$temp"
        fi
    else
        install -m "$mode" -o "$owner" -g "$group" -- "$source" "$temp"
    fi
    snapshot_target "$target"
    mv -fT -- "$temp" "$target"
}

atomic_install_symlink() {
    local link_target="$1" link_path="$2" dir base temp
    dir="$(dirname -- "$link_path")"
    base="$(basename -- "$link_path")"
    mkdir -p -- "$dir"
    temp="${dir}/.${base}.tmp.$$"
    rm -f -- "$temp"
    if [[ -n "$DEPLOY_ROOT" && "${DEPLOY_OFFLINE_TEST_RELAX_SECRET_MODE:-0}" == "1" ]]; then
        # Git Bash/NTFS 无符号链接权限时用内容标记完成幂等文件测试；Ubuntu CI 走真实链接。
        printf '%s\n' "$link_target" > "$temp"
    else
        ln -s -- "$link_target" "$temp"
    fi
    snapshot_target "$link_path"
    mv -fT -- "$temp" "$link_path"
}

ensure_directory() {
    local path="$1" owner="$2" group="$3" mode="$4" i
    for ((i=0; i<${#DIR_PATHS[@]}; i++)); do
        [[ "${DIR_PATHS[$i]}" == "$path" ]] && return 0
    done
    DIR_PATHS+=("$path")
    if [[ -d "$path" && ! -L "$path" ]]; then
        DIR_EXISTED+=("1")
        DIR_UIDS+=("$(stat -c '%u' -- "$path")")
        DIR_GIDS+=("$(stat -c '%g' -- "$path")")
        DIR_MODES+=("$(stat -c '%a' -- "$path")")
    elif [[ -e "$path" || -L "$path" ]]; then
        die "目标目录被非目录或符号链接占用: $path"
    else
        DIR_EXISTED+=("0")
        DIR_UIDS+=("0")
        DIR_GIDS+=("0")
        DIR_MODES+=("0")
    fi
    if [[ -n "$DEPLOY_ROOT" ]]; then
        mkdir -p -- "$path"
        if [[ "${DEPLOY_OFFLINE_TEST_RELAX_SECRET_MODE:-0}" != "1" ]]; then
            chmod "$mode" -- "$path"
        fi
    else
        install -d -m "$mode" -o "$owner" -g "$group" -- "$path"
    fi
}

log "=== 2/7 准备非破坏 FTP 目录和账号 ==="
if [[ -n "$DEPLOY_ROOT" ]]; then
    account_exists devicedata || CREATED_DEVICEDATA=1
else
    if ! getent group ftpdata >/dev/null 2>&1; then
        create_ftpdata_group
    fi
    validate_ftpdata_group
    if ! id devicedata >/dev/null 2>&1; then
        create_ftp_account devicedata
    else
        [[ -z "$DEVICEDATA_PASSWORD_FILE" ]] ||
            die "devicedata 在部署期间变为既有账号，拒绝继续或改密"
        validate_existing_ftp_account devicedata
        add_existing_account_to_ftpdata devicedata
    fi
    validate_existing_ftp_account devicedata
    validate_devicedata_private_group
fi
if [[ -z "$DEPLOY_ROOT" || -n "$DEPLOY_NSS_FIXTURE_DIR" ]]; then
    validate_ftpdata_group
    validate_ftpdata_membership_closure
fi

ensure_directory "$FTP_ROOT" root root 0755
ensure_directory "$FTP_DATA" root devicedata 2771
ensure_directory "$VSFTPD_USER_CONF_DIR" root root 0755
ensure_directory "$OTA_ROOT" root root 0755
ensure_directory "${OTA_ROOT}/neutral" root root 0755
ensure_directory "${OTA_ROOT}/brand" root root 0755
ensure_directory "$OTA_ADMIN_DIR" root root 0700
if [[ -n "$DEPLOY_ROOT" ]]; then
    ensure_directory "$(dirname -- "$OFFLINE_ACCOUNT_STATE")" root root 0700
    append_line_candidate "$OFFLINE_ACCOUNT_STATE" "${STAGE_DIR}/offline-accounts" "devicedata"
    atomic_install_file "${STAGE_DIR}/offline-accounts" "$OFFLINE_ACCOUNT_STATE" 0600 root root
fi

log "=== 3/7 原子安装 nginx/vsftpd 配置 ==="
atomic_install_file "${STAGE_DIR}/nginx-ota.conf" "$NGINX_SITE" 0644 root root
atomic_install_symlink /etc/nginx/sites-available/ota "$NGINX_LINK"
atomic_install_file "${STAGE_DIR}/vsftpd.conf" "$VSFTPD_CONFIG" 0600 root root
atomic_install_file "${STAGE_DIR}/vsftpd.userlist" "$VSFTPD_USERLIST" 0600 root root
atomic_install_file "${STAGE_DIR}/shells" "$SHELLS_FILE" 0644 root root
verify_userlist_preserved "${STAGE_DIR}/original-vsftpd.userlist" "$VSFTPD_USERLIST"
# 共享 uploader 已不在 allow-list；删除其旧按用户文件，避免后续误判为可恢复账号。
if [[ -e "${VSFTPD_USER_CONF_DIR}/uploader" || -L "${VSFTPD_USER_CONF_DIR}/uploader" ]]; then
    [[ -f "${VSFTPD_USER_CONF_DIR}/uploader" && ! -L "${VSFTPD_USER_CONF_DIR}/uploader" ]] ||
        die "退役 uploader 配置不是普通文件"
    snapshot_target "${VSFTPD_USER_CONF_DIR}/uploader"
    rm -f -- "${VSFTPD_USER_CONF_DIR}/uploader"
fi
# 无按用户覆盖即为 full。任何旧配置都会先备份，再移除；失败回滚时恢复。
if [[ -e "${VSFTPD_USER_CONF_DIR}/devicedata" || -L "${VSFTPD_USER_CONF_DIR}/devicedata" ]]; then
    [[ -f "${VSFTPD_USER_CONF_DIR}/devicedata" && ! -L "${VSFTPD_USER_CONF_DIR}/devicedata" ]] ||
        die "devicedata 用户配置不是普通文件，拒绝自动移除"
    snapshot_target "${VSFTPD_USER_CONF_DIR}/devicedata"
    rm -f -- "${VSFTPD_USER_CONF_DIR}/devicedata"
fi
if [[ -z "$DEPLOY_ROOT" ]]; then
    id -nG devicedata | tr ' ' '\n' | grep -Fqx ftpdata ||
        die "devicedata 未加入 ftpdata 共享组"
    [[ "$(stat -c '%a' -- "$FTP_DATA")" == "2771" ]] ||
        die "数据根目录未保持 2771（管理员 rwx、设备仅 traverse）"
    admin_gid="$(lookup_passwd_entry devicedata | awk -F: '{print $4}')"
    [[ "$(stat -c '%g' -- "$FTP_DATA")" == "$admin_gid" ]] ||
        die "数据根目录组不是 devicedata 私有主组"
fi
[[ ! -e "${VSFTPD_USER_CONF_DIR}/devicedata" ]] ||
    die "devicedata 应为 full，不能残留按用户限制"
if [[ -z "$DEPLOY_ROOT" || -n "$DEPLOY_NSS_FIXTURE_DIR" ]]; then
    validate_final_ftp_account_state
fi

log "=== 4/7 安装 ota-admin 与清理任务 ==="
[[ "$(sha256sum -- "$OTA_ADMIN_STAGED" | awk '{print $1}')" == "$OTA_ADMIN_STAGED_HASH" ]] ||
    die "staged ota_admin.py 在校验后发生变化"
atomic_install_file "$OTA_ADMIN_STAGED" "$OTA_ADMIN_TARGET" 0755 root root
[[ "$(sha256sum -- "$OTA_ADMIN_TARGET" | awk '{print $1}')" == "$OTA_ADMIN_STAGED_HASH" ]] ||
    die "ota_admin.py 安装后哈希不一致"
atomic_install_file "${STAGE_DIR}/ota-admin.service" "$OTA_ADMIN_UNIT" 0644 root root
atomic_install_file "${STAGE_DIR}/clean-devicedata" "$CLEANUP_CRON" 0755 root root

if [[ -n "$ADMIN_TOKEN_SOURCE" ]]; then
    atomic_install_file "$ADMIN_TOKEN_SOURCE" "$OTA_ADMIN_TOKEN" 0600 root root
elif [[ -f "$OTA_ADMIN_TOKEN" && ! -L "$OTA_ADMIN_TOKEN" ]]; then
    validate_secret_file "$OTA_ADMIN_TOKEN" "既有管理令牌" 32 0
    cp -- "$OTA_ADMIN_TOKEN" "${STAGE_DIR}/existing-token"
    if [[ -z "$DEPLOY_ROOT" || "${DEPLOY_OFFLINE_TEST_RELAX_SECRET_MODE:-0}" != "1" ]]; then
        chmod 0600 -- "${STAGE_DIR}/existing-token"
    fi
    atomic_install_file "${STAGE_DIR}/existing-token" "$OTA_ADMIN_TOKEN" 0600 root root
elif [[ -e "$OTA_ADMIN_TOKEN" || -L "$OTA_ADMIN_TOKEN" ]]; then
    die "管理令牌目标不是普通文件: $OTA_ADMIN_TOKEN"
else
    umask 077
    python3 -I - <<'PY' > "${STAGE_DIR}/generated-token"
import secrets
print(secrets.token_hex(32))
PY
    atomic_install_file "${STAGE_DIR}/generated-token" "$OTA_ADMIN_TOKEN" 0600 root root
fi

log "=== 5/7 验证配置并启动服务 ==="
if [[ -z "$DEPLOY_ROOT" ]]; then
    nginx -t
    if command -v systemd-analyze >/dev/null 2>&1; then
        systemd-analyze verify "$OTA_ADMIN_UNIT"
    fi
    # 明文 FTP 不得在防火墙边界确认前启动。规则即使后续事务回滚也只会更收紧。
    ensure_private_ftp_firewall
    systemctl daemon-reload
    systemctl enable --now nginx vsftpd ota-admin
    systemctl reload nginx
    systemctl restart vsftpd ota-admin
    systemctl is-active --quiet nginx
    systemctl is-active --quiet vsftpd
    systemctl is-active --quiet ota-admin

    OTA_ADMIN_TOKEN_FILE="$OTA_ADMIN_TOKEN" python3 -I - <<'PY'
import json
import os
import socket
import time
import urllib.request

token = open(os.environ["OTA_ADMIN_TOKEN_FILE"], "r", encoding="utf-8").read().strip()

for url in (
    "http://127.0.0.1:8091/admin/api/ping",
    "http://127.0.0.1:8090/admin/api/ping",
):
    request = urllib.request.Request(url, headers={"X-Admin-Token": token})
    last_error = None
    for _ in range(20):
        try:
            with urllib.request.urlopen(request, timeout=2) as response:
                body = json.loads(response.read().decode("utf-8"))
                if response.status == 200 and body.get("ok") is True:
                    break
        except Exception as exc:
            last_error = exc
            time.sleep(0.25)
    else:
        raise SystemExit(f"本机管理接口健康检查失败 ({url}): {last_error}")

last_error = None
for _ in range(20):
    try:
        with socket.create_connection(("127.0.0.1", 21), timeout=2) as connection:
            banner = connection.recv(128)
        if banner.startswith(b"220"):
            break
        last_error = RuntimeError("FTP banner 不是 220")
    except Exception as exc:
        last_error = exc
        time.sleep(0.25)
else:
    raise SystemExit(f"vsftpd 本机端口健康检查失败: {last_error}")
PY
else
    log "离线测试根目录：跳过 systemd/nginx 运行态操作。"
fi
log "=== 6/7 设置可选 FTP 密码 ==="
if [[ -n "$DEPLOY_ROOT" ]]; then
    # 密码文件已完成格式/权限校验；离线模式严禁调用宿主机 chpasswd，也不落盘秘密。
    [[ "$CREATED_DEVICEDATA" -eq 0 || -n "$DEVICEDATA_PASSWORD_FILE" ]] ||
        die "内部错误: 离线新建 devicedata 缺少密码文件"
    log "离线测试：devicedata 密码输入已校验，未调用宿主机 chpasswd。"
else
    if [[ "$CREATED_DEVICEDATA" -eq 0 ]]; then
        [[ -z "$DEVICEDATA_PASSWORD_FILE" ]] || die "内部错误: 既有 devicedata 收到改密文件"
        log "既有 devicedata 保留原密码；部署器不承担密码轮换。"
    fi
    if [[ "$CREATED_DEVICEDATA" -eq 1 ]]; then
        # 单账号 chpasswd 使用 10s 阶段预算；秘密只经匿名管道传递，不落盘/参数/日志。
        set +e
        {
            if [[ "$CREATED_DEVICEDATA" -eq 1 ]]; then
                IFS= read -r devicedata_password < "$DEVICEDATA_PASSWORD_FILE" || true
                printf 'devicedata:%s\n' "$devicedata_password"
                unset devicedata_password
            fi
        } | timeout --signal=TERM --kill-after=5s 10s chpasswd
        password_status=("${PIPESTATUS[@]}")
        set -e
        [[ "${password_status[0]}" -eq 0 && "${password_status[1]}" -eq 0 ]] ||
            die "新账号初始密码设置失败或超时"
        unset password_status
    fi
fi

# 配置、健康检查和密码均已完成，必须先提交再释放共享锁；这样释放后的 admin 新
# mutation 不会被迟到的 TERM/INT 当成“未提交部署”回滚或中断。
TRANSACTION_COMMITTED=1
if [[ -z "$DEPLOY_ROOT" ]]; then
    release_account_mutation_lock
fi

log "=== 7/7 回读强制防火墙边界 ==="
if [[ -n "$DEPLOY_ROOT" ]]; then
    log "离线测试：已验证强制 UFW 参数/规则夹具，未调用宿主机 ufw。"
else
    verify_final_private_ftp_firewall
fi

log ""
log "部署和本机验证完成。"
log "备份目录: $BACKUP_DIR"
log "管理令牌仅保存在受限文件 /opt/ota-admin/token；本脚本不会显示其内容。"
log "管理接口默认仅允许 loopback；远程管理必须经 TLS 或 SSH 等可信隧道。"
log "共享 uploader 已退役；普通账号只写 /data/<account>，devicedata 通过私有组管理。"
log "容量边界: ${CAPACITY_STATUS}；当前无 per-device quota。"
