#!/usr/bin/env bash
# deploy_online_services.sh 的纯离线回归测试：只构造临时根目录并执行 dry-run。
set -Eeuo pipefail

readonly TEST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REPO_ROOT="$(cd -- "${TEST_DIR}/../.." && pwd -P)"
readonly DEPLOY_SCRIPT="${REPO_ROOT}/scripts/server/deploy_online_services.sh"

fail() {
    echo "FAIL: $*" >&2
    exit 1
}

assert_contains() {
    local text="$1" expected="$2"
    [[ "$text" == *"$expected"* ]] || fail "输出缺少: $expected"
}

root="$(mktemp -d "${TMPDIR:-/tmp}/deploy-online-services-test.XXXXXX")"
before="${root}.before"
apply_root="$(mktemp -d "${TMPDIR:-/tmp}/deploy-online-services-apply.XXXXXX")"
secret_dir="$(mktemp -d "${TMPDIR:-/tmp}/deploy-online-services-secrets.XXXXXX")"
managed_before="$(mktemp "${TMPDIR:-/tmp}/deploy-managed-before.XXXXXX")"
managed_after="$(mktemp "${TMPDIR:-/tmp}/deploy-managed-after.XXXXXX")"
nss_fixture="$(mktemp -d "${TMPDIR:-/tmp}/deploy-nss-fixture.XXXXXX")"
ufw_fixture="$(mktemp "${TMPDIR:-/tmp}/deploy-ufw-rules.XXXXXX")"
capacity_fixture="$(mktemp "${TMPDIR:-/tmp}/deploy-capacity.XXXXXX")"
cleanup_root="$(mktemp -d "${TMPDIR:-/tmp}/deploy-cleanup-data.XXXXXX")"
preflight_root="$(mktemp -d "${TMPDIR:-/tmp}/deploy-preflight-root.XXXXXX")"
missing_path="$(mktemp -d "${TMPDIR:-/tmp}/deploy-missing-path.XXXXXX")"
fake_path="$(mktemp -d "${TMPDIR:-/tmp}/deploy-fake-path.XXXXXX")"
fake_path_sentinel="${fake_path}/invoked"
python_hijack_dir="$(mktemp -d "${TMPDIR:-/tmp}/deploy-python-hijack.XXXXXX")"
python_hijack_sentinel="${python_hijack_dir}/imported"
preflight_before="${preflight_root}.before"
trap 'rm -rf -- "$root" "$before" "$apply_root" "$secret_dir" "$nss_fixture" "$cleanup_root" "$preflight_root" "$preflight_before" "$missing_path" "$fake_path" "$python_hijack_dir"; rm -f -- "$managed_before" "$managed_after" "$ufw_fixture" "$capacity_fixture"' EXIT
printf '%s\n' '{"mountTarget":"/srv/devicedata","totalBytes":1073741824,"mountSource":"/dev/loop7","mountIdentity":"7:7","parentIdentity":"8:1","fsType":"ext4","mountOptions":"rw,relatime"}' > "$capacity_fixture"

readonly offline_root_marker='NO_TEACHING_ONLINE_SERVICES_OFFLINE_ROOT_V1'
for offline_root in "$root" "$apply_root" "$preflight_root"; do
    printf '%s\n' "$offline_root_marker" > \
        "${offline_root}/.no-teaching-online-services-offline-root"
done
mkdir -p -- "${preflight_root}/child"
cp -a -- "$preflight_root" "$preflight_before"
bash_bin="$(command -v bash)"

host_deploy_metadata() {
    local path
    for path in /etc/nginx/sites-available/ota /etc/vsftpd.conf /etc/vsftpd.userlist \
        /opt/ota-admin /var/backups/no-teaching-online-services /var/www/ota /srv/devicedata; do
        if [[ -e "$path" || -L "$path" ]]; then
            stat -c '%n|%F|%s|%Y|%i|%a|%u|%g' -- "$path"
        else
            printf '%s|absent\n' "$path"
        fi
    done
}

# 测试根目录必须是带 marker 的 canonical 非根目录；危险拼法在任何目标计算前拒绝。
host_metadata_before="$(host_deploy_metadata)"
for invalid_root in / '////' "${preflight_root}/child/.."; do
    set +e
    invalid_root_output="$(
        DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$invalid_root" \
            "$bash_bin" "$DEPLOY_SCRIPT" --apply --yes --skip-packages 2>&1
    )"
    invalid_root_rc=$?
    set -e
    [[ $invalid_root_rc -ne 0 ]] || fail "危险 DEPLOY_ROOT 未被拒绝: $invalid_root"
    assert_contains "$invalid_root_output" "DEPLOY_ROOT"
done
host_metadata_after="$(host_deploy_metadata)"
[[ "$host_metadata_before" == "$host_metadata_after" ]] ||
    fail "拒绝危险 DEPLOY_ROOT 时宿主托管目标元数据发生变化"
diff -ru -- "$preflight_before" "$preflight_root" >/dev/null ||
    fail "拒绝危险 DEPLOY_ROOT 时改动了离线夹具"

# apply 必须显式确认依赖已在隔离维护窗口预装，且门禁失败不得创建 staging/备份。
set +e
missing_ack_output="$(
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$preflight_root" \
        bash "$DEPLOY_SCRIPT" --apply --yes 2>&1
)"
missing_ack_rc=$?
set -e
[[ $missing_ack_rc -ne 0 ]] || fail "--apply 未给 --skip-packages 仍继续执行"
assert_contains "$missing_ack_output" "--apply 必须显式给出 --skip-packages"
diff -ru -- "$preflight_before" "$preflight_root" >/dev/null ||
    fail "依赖预装确认失败后仍写入了部署根目录"

# 用空 PATH 模拟缺失核心依赖；离线 marker 校验使用可信绝对路径完成后才恢复这个
# 测试 PATH，因此脚本应在 CIDR 所需 python3 处 fail closed，不得进入 staging。
set +e
missing_dependency_output="$(
    PATH="$missing_path" DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$preflight_root" \
        DEPLOY_CAPACITY_FIXTURE="$capacity_fixture" \
        "$bash_bin" "$DEPLOY_SCRIPT" --apply --yes --skip-packages \
        --enable-ufw --ftp-allow-cidr 10.20.0.0/16 \
        --data-filesystem-max-bytes 1073741824 2>&1
)"
missing_dependency_rc=$?
set -e
[[ $missing_dependency_rc -ne 0 ]] || fail "缺少预装依赖时 apply 仍继续执行"
assert_contains "$missing_dependency_output" "CIDR 校验需要 python3"
diff -ru -- "$preflight_before" "$preflight_root" >/dev/null ||
    fail "预装依赖缺失后仍写入了部署根目录"

# 同源 bind 即使 TARGET 恰好是 /srv/devicedata 也不构成独立容量边界。
printf '%s\n' '{"mountTarget":"/srv/devicedata","totalBytes":1073741824,"mountSource":"/dev/sda1","mountIdentity":"8:1","parentIdentity":"8:1","fsType":"ext4","mountOptions":"rw,bind"}' > "$capacity_fixture"
set +e
bind_capacity_output="$(
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$preflight_root" \
        DEPLOY_CAPACITY_FIXTURE="$capacity_fixture" \
        "$bash_bin" "$DEPLOY_SCRIPT" --apply --yes --skip-packages \
        --enable-ufw --ftp-allow-cidr 10.20.0.0/16 \
        --data-filesystem-max-bytes 1073741824 2>&1
)"
bind_capacity_rc=$?
set -e
[[ $bind_capacity_rc -ne 0 ]] || fail "同源 bind 容量夹具未被拒绝"
assert_contains "$bind_capacity_output" "bind"
printf '%s\n' '{"mountTarget":"/srv/devicedata","totalBytes":1073741824,"mountSource":"/dev/loop7","mountIdentity":"7:7","parentIdentity":"8:1","fsType":"ext4","mountOptions":"rw,relatime"}' > "$capacity_fixture"

# 普通/生产 dry-run 即使带测试 flag 但没有隔离根目录，也必须覆盖调用方 PATH；
# 任何伪造的 basename/dirname/awk/python3 都不得在可信 PATH 门禁前被执行。
for command_name in basename dirname awk python3; do
    {
        printf '#!%s\n' "$bash_bin"
        printf 'printf "invoked\\n" >> "$FAKE_PATH_SENTINEL"\n'
        printf 'exit 97\n'
    } > "${fake_path}/${command_name}"
    chmod 0755 "${fake_path}/${command_name}"
done
set +e
trusted_path_output="$(
    PATH="$fake_path" FAKE_PATH_SENTINEL="$fake_path_sentinel" DEPLOY_OFFLINE_TEST=1 \
        "$bash_bin" "$DEPLOY_SCRIPT" 2>&1
)"
trusted_path_rc=$?
set -e
[[ ! -e "$fake_path_sentinel" ]] || fail "普通/生产路径执行了调用方 PATH 中的伪造命令"
# 实机是否已预装 nginx/vsftpd 会决定此 dry-run 成败；本用例只验证不信任 PATH。
: "$trusted_path_rc" "$trusted_path_output"

mkdir -p -- \
    "${root}/etc/nginx/sites-available" \
    "${root}/etc/nginx/sites-enabled" \
    "${root}/etc/vsftpd_user_conf"

cat > "${root}/etc/vsftpd.userlist" <<'EOF'
legacy-camera
devicedata
uploader
EOF

cat > "${root}/etc/vsftpd.conf" <<'EOF'
# 必须保留的非托管配置
log_ftp_protocol=YES
local_umask=002
EOF

cat > "${root}/etc/vsftpd_user_conf/uploader" <<'EOF'
# 必须保留的额外限制
hide_ids=YES
cmds_denied=SITE_CHMOD
EOF

cat > "${root}/etc/shells" <<'EOF'
/bin/sh
/usr/sbin/nologin
EOF

cp -a -- "$root" "$before"

output="$(
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$root" \
        bash "$DEPLOY_SCRIPT" --skip-packages --configure-ufw --ssh-port 48890 \
        --ftp-allow-cidr 10.20.0.0/16 \
        --pasv-address ftp.example.test
)"
assert_contains "$output" "预演（未修改任何文件）"
assert_contains "$output" "保留既有 FTP 白名单"
assert_contains "$output" "ftpoperator=FTP 上传下载"
assert_contains "$output" "uploader=共享 write-only"
assert_contains "$output" "FTP 仅允许 10.20.0.0/16"
assert_contains "$output" "PASV 公网回址: 显式设置为 ftp.example.test"
diff -ru -- "$before" "$root" >/dev/null || fail "dry-run 修改了临时根目录"

# root 从用户可写 cwd 启动且继承 PYTHONPATH/PYTHONHOME 时，inline Python 必须使用
# isolated mode，不能导入同名 ipaddress.py。
cat > "${python_hijack_dir}/ipaddress.py" <<'PY'
import os
with open(os.environ["PYTHON_HIJACK_SENTINEL"], "w", encoding="utf-8") as handle:
    handle.write("imported\n")
raise RuntimeError("cwd module hijack")
PY
python_hijack_output="$(
    cd -- "$python_hijack_dir"
    PYTHONPATH="$python_hijack_dir" PYTHONHOME="$python_hijack_dir" \
        PYTHON_HIJACK_SENTINEL="$python_hijack_sentinel" \
        DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$root" \
        "$bash_bin" "$DEPLOY_SCRIPT" --skip-packages \
        --ftp-allow-cidr 10.20.0.0/16
)"
assert_contains "$python_hijack_output" "预演（未修改任何文件）"
[[ ! -e "$python_hijack_sentinel" ]] || fail "inline Python 导入了 cwd/PYTHONPATH 恶意模块"
diff -ru -- "$before" "$root" >/dev/null || fail "Python 隔离 dry-run 修改了临时根目录"

warning_output="$(
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$root" \
        bash "$DEPLOY_SCRIPT" --skip-packages
)"
assert_contains "$warning_output" "未配置；NAT/公网服务器的被动连接可能失败"

# 全局 vsftpd 未知正向/高影响指令不得从旧配置漂移进新服务。
cp -- "${root}/etc/vsftpd.conf" "${root}/etc/vsftpd.conf.safe"
printf '%s\n' 'cmds_allowed=STOR,MKD' >> "${root}/etc/vsftpd.conf"
set +e
global_unsafe_output="$(
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$root" \
        bash "$DEPLOY_SCRIPT" --skip-packages 2>&1
)"
global_unsafe_rc=$?
set -e
[[ $global_unsafe_rc -ne 0 ]] || fail "全局 cmds_allowed 漂移未被拒绝"
assert_contains "$global_unsafe_output" "cmds_allowed"
mv -f -- "${root}/etc/vsftpd.conf.safe" "${root}/etc/vsftpd.conf"
for restrictive_directive in \
    'download_enable=NO' \
    'chmod_enable=NO' \
    'file_open_mode=0600' \
    'local_umask=077'; do
    cp -- "${root}/etc/vsftpd.conf" "${root}/etc/vsftpd.conf.safe"
    if [[ "$restrictive_directive" == local_umask=* ]]; then
        sed -i "s/^local_umask=.*/${restrictive_directive}/" "${root}/etc/vsftpd.conf"
    else
        printf '%s\n' "$restrictive_directive" >> "${root}/etc/vsftpd.conf"
    fi
    set +e
    restrictive_output="$(
        DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$root" \
            bash "$DEPLOY_SCRIPT" --skip-packages 2>&1
    )"
    restrictive_rc=$?
    set -e
    [[ $restrictive_rc -ne 0 ]] ||
        fail "既有安全限制被静默放宽: $restrictive_directive"
    assert_contains "$restrictive_output" "必须为"
    mv -f -- "${root}/etc/vsftpd.conf.safe" "${root}/etc/vsftpd.conf"
done

# 默认账号秘密不进入部署器参数；统一经回环管理接口轮换。
set +e
retired_uploader_output="$(
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$root" \
        bash "$DEPLOY_SCRIPT" --uploader-password-file /tmp/retired 2>&1
)"
retired_uploader_rc=$?
set -e
[[ $retired_uploader_rc -ne 0 ]] || fail "uploader 密码参数仍被部署器接受"
assert_contains "$retired_uploader_output" "不由部署器接收"

# 命令行明文秘密必须被当成未知位置参数拒绝。
set +e
plaintext_output="$(bash "$DEPLOY_SCRIPT" 'plain-text-password' 2>&1)"
plaintext_rc=$?
set -e
[[ $plaintext_rc -ne 0 ]] || fail "明文位置参数未被拒绝"
assert_contains "$plaintext_output" "不接受明文 FTP 密码"

# SSH 端口必须经过范围校验；UFW 本身仍然只有显式选项才会进入计划。
set +e
port_output="$(bash "$DEPLOY_SCRIPT" --ssh-port 70000 2>&1)"
port_rc=$?
set -e
[[ $port_rc -ne 0 ]] || fail "非法 SSH 端口未被拒绝"
assert_contains "$port_output" "1-65535"

set +e
pasv_output="$(bash "$DEPLOY_SCRIPT" --pasv-address 'bad/address' 2>&1)"
pasv_rc=$?
set -e
[[ $pasv_rc -ne 0 ]] || fail "非法 PASV 回址未被拒绝"
assert_contains "$pasv_output" "--pasv-address"

set +e
missing_cidr_output="$(bash "$DEPLOY_SCRIPT" --configure-ufw 2>&1)"
missing_cidr_rc=$?
set -e
[[ $missing_cidr_rc -ne 0 ]] || fail "未提供 CIDR 时仍允许配置 FTP 防火墙"
assert_contains "$missing_cidr_output" "拒绝把 FTP 暴露到公网"

set +e
invalid_cidr_output="$(
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$root" \
        bash "$DEPLOY_SCRIPT" --configure-ufw --ftp-allow-cidr 10.20.0.1/16 2>&1
)"
invalid_cidr_rc=$?
set -e
[[ $invalid_cidr_rc -ne 0 ]] || fail "含主机位的 FTP CIDR 未被拒绝"
assert_contains "$invalid_cidr_output" "--ftp-allow-cidr"
for forbidden_cidr in 0.0.0.0/0 8.8.8.0/24 ::/0; do
    set +e
    forbidden_cidr_output="$(
        DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$root" \
            bash "$DEPLOY_SCRIPT" --configure-ufw --ftp-allow-cidr "$forbidden_cidr" 2>&1
    )"
    forbidden_cidr_rc=$?
    set -e
    [[ $forbidden_cidr_rc -ne 0 ]] || fail "公网/全网 FTP CIDR 未被拒绝: $forbidden_cidr"
    assert_contains "$forbidden_cidr_output" "仅允许 RFC1918"
done

run_ufw_rules_fixture() {
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$root" DEPLOY_UFW_RULES_FIXTURE="$ufw_fixture" \
        bash "$DEPLOY_SCRIPT" --skip-packages --configure-ufw \
        --ftp-allow-cidr 10.20.0.0/16 --pasv-address ftp.example.test
}
for unsafe_rule in \
    'ufw allow from any' \
    'ufw allow 1:65535/tcp' \
    'ufw allow 20:21/tcp' \
    'ufw allow ftp'; do
    printf '%s\n' "$unsafe_rule" > "$ufw_fixture"
    set +e
    unsafe_ufw_output="$(run_ufw_rules_fixture 2>&1)"
    unsafe_ufw_rc=$?
    set -e
    [[ $unsafe_ufw_rc -ne 0 ]] || fail "不确定/更宽 UFW allow 未被拒绝: $unsafe_rule"
done
cat > "$ufw_fixture" <<'EOF'
ufw allow 48890/tcp
ufw allow from 10.20.0.0/16 to any port 21 proto tcp
ufw allow from 10.20.0.0/16 to any port 40000:40100 proto tcp
EOF
safe_ufw_output="$(run_ufw_rules_fixture)"
assert_contains "$safe_ufw_output" "预演（未修改任何文件）"
printf '%s\n' 'ufw allow 48890/tcp' > "$ufw_fixture"
unrelated_ufw_output="$(run_ufw_rules_fixture)"
assert_contains "$unrelated_ufw_output" "预演（未修改任何文件）"

grep -Fq 'SSH_PORT=48890' "$DEPLOY_SCRIPT" || fail "默认 SSH 端口漂移"
grep -Fq 'local_umask"]="002"' "$DEPLOY_SCRIPT" || fail "ftpdata 共享 umask 漂移"
grep -Fq 'ensure_directory "$FTP_DATA" root devicedata 2771' "$DEPLOY_SCRIPT" ||
    fail "数据根目录 traverse-only/devicedata 私有组门禁漂移"
grep -Fq 'local_root 必须保持 root-owned /srv/devicedata chroot' "$DEPLOY_SCRIPT" ||
    fail "root-owned chroot 门禁缺失"
if grep -Eq 'rm[[:space:]]+-f[[:space:]]+/etc/nginx/sites-enabled/default' "$DEPLOY_SCRIPT"; then
    fail "部署脚本仍会删除其他 nginx 站点"
fi
if grep -Eq 'echo[[:space:]]+.*>[[:space:]]*/etc/vsftpd\.userlist' "$DEPLOY_SCRIPT"; then
    fail "部署脚本仍会覆盖 FTP 白名单"
fi

# 按 POSIX 内核 owner/group/other 选择规则验证最终访问矩阵：设备 A 能 traverse
# root/data 并写自己的目录，但对 B 目录无 stat/list/write 权限；管理员组可访问二者。
python3 -I - <<'PY'
def bits(mode, owner, group, uid, groups):
    shift = 6 if uid == owner else (3 if group in groups else 0)
    return (mode >> shift) & 0o7

ftp_root = (0o755, 0, 0)
data_root = (0o2771, 0, 2000)
device_a = (0o2770, 1002, 2000)
device_b = (0o2770, 1003, 2000)
uid_a, groups_a = 1002, {995, 5002}
uid_admin, groups_admin = 1000, {995, 2000}
assert bits(*ftp_root, uid_a, groups_a) & 0o1
assert bits(*data_root, uid_a, groups_a) == 0o1
assert bits(*device_a, uid_a, groups_a) == 0o7
assert bits(*device_b, uid_a, groups_a) == 0
assert bits(*device_a, uid_admin, groups_admin) == 0o7
assert bits(*device_b, uid_admin, groups_admin) == 0o7
PY

# 真正执行到隔离根目录，证明 apply 不会触碰宿主账号/服务/防火墙，并可重复执行。
mkdir -p -- \
    "${apply_root}/etc/nginx/sites-available" \
    "${apply_root}/etc/nginx/sites-enabled" \
    "${apply_root}/etc/vsftpd_user_conf"
cat > "${apply_root}/etc/vsftpd.userlist" <<'EOF'
legacy-camera
EOF
cat > "${apply_root}/etc/vsftpd.conf" <<'EOF'
# 非托管配置必须跨两次 apply 保留
log_ftp_protocol=YES
local_umask=002
EOF
cat > "${apply_root}/etc/vsftpd_user_conf/uploader" <<'EOF'
hide_ids=YES
cmds_denied=SITE_CHMOD
EOF
cat > "${apply_root}/etc/shells" <<'EOF'
/bin/sh
EOF
printf '%s\n' 'offline-devicedata-password' > "${secret_dir}/devicedata.password"
printf '%s\n' 'offline-admin-token-0123456789abcdef' > "${secret_dir}/admin.token"
chmod 0600 -- "${secret_dir}/devicedata.password" \
    "${secret_dir}/admin.token"
supports_posix_modes=1
if [[ "$(stat -c '%a' -- "${secret_dir}/devicedata.password")" != "600" ]]; then
    # Git Bash on NTFS may ignore chmod; the Ubuntu CI path keeps the strict 0600 assertion.
    supports_posix_modes=0
fi

host_accounts_before="$(
    { id devicedata 2>&1 || true; id uploader 2>&1 || true; }
)"

apply_once() {
    local initialize_accounts="${1:-0}"
    local enable_xtrace="${2:-0}"
    local -a args=(
        --apply --yes --skip-packages
        --configure-ufw --enable-ufw --ssh-port 48890 \
        --ftp-allow-cidr 10.20.0.0/16 \
        --data-filesystem-max-bytes 1073741824 \
        --pasv-address ftp.example.test
    )
    if [[ "$initialize_accounts" -eq 1 ]]; then
        args+=(
            --devicedata-password-file "${secret_dir}/devicedata.password"
            --admin-token-file "${secret_dir}/admin.token"
        )
    fi
    local -a bash_args=()
    [[ "$enable_xtrace" -eq 0 ]] || bash_args+=(-x)
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$apply_root" DEPLOY_CAPACITY_FIXTURE="$capacity_fixture" \
        DEPLOY_OFFLINE_TEST_RELAX_SECRET_MODE="$((1 - supports_posix_modes))" \
        "$bash_bin" "${bash_args[@]}" "$DEPLOY_SCRIPT" "${args[@]}"
}

managed_fingerprint() {
    local target_root="$1"
    (
        cd -- "$target_root"
        for path in \
            etc/nginx/sites-available/ota \
            etc/nginx/sites-enabled/ota \
            etc/vsftpd.conf \
            etc/vsftpd.userlist \
            etc/shells \
            etc/systemd/system/ota-admin.service \
            etc/cron.daily/clean-devicedata \
            opt/ota-admin/ota_admin.py \
            opt/ota-admin/token \
            var/lib/no-teaching-online-services/test-accounts; do
            if [[ -L "$path" ]]; then
                printf 'link %s %s\n' "$path" "$(readlink -- "$path")"
            else
                stat -c 'file %a %n' -- "$path"
                sha256sum -- "$path"
            fi
        done
        stat -c 'dir %a %n' -- srv/devicedata srv/devicedata/data \
            etc/vsftpd_user_conf var/www/ota var/www/ota/neutral var/www/ota/brand
    )
}

first_apply_output="$(apply_once 1 1 2>&1)"
assert_contains "$first_apply_output" "离线测试根目录"
assert_contains "$first_apply_output" "未调用宿主机 chpasswd"
assert_contains "$first_apply_output" "未调用宿主机 ufw"
for secret_value in offline-devicedata-password \
    offline-admin-token-0123456789abcdef; do
    [[ "$first_apply_output" != *"$secret_value"* ]] ||
        fail "bash -x/-v 输出泄露了秘密文件内容"
done
for unit_directive in \
    'RuntimeDirectory=no-teaching-ota' \
    'RuntimeDirectoryMode=0700' \
    'RuntimeDirectoryPreserve=yes'; do
    grep -Fqx "$unit_directive" "${apply_root}/etc/systemd/system/ota-admin.service" ||
        fail "ota-admin unit 缺少安全运行目录配置: $unit_directive"
done

for account in legacy-camera devicedata; do
    [[ "$(grep -Fxc -- "$account" "${apply_root}/etc/vsftpd.userlist")" == "1" ]] ||
        fail "白名单账号缺失或重复: $account"
done
! grep -Fqx uploader "${apply_root}/etc/vsftpd.userlist" || fail "退役 uploader 仍在白名单"
[[ ! -e "${apply_root}/etc/vsftpd_user_conf/uploader" ]] || fail "退役 uploader 配置仍存在"
for account in devicedata; do
    [[ "$(grep -Fxc -- "$account" "${apply_root}/var/lib/no-teaching-online-services/test-accounts")" == "1" ]] ||
        fail "离线账号状态缺失或重复: $account"
done
grep -Fqx 'log_ftp_protocol=YES' "${apply_root}/etc/vsftpd.conf" ||
    fail "非托管 vsftpd 配置丢失"
grep -Fqx 'local_umask=002' "${apply_root}/etc/vsftpd.conf" || fail "共享 umask 未安装"
grep -Fqx 'user_config_dir=/etc/vsftpd_user_conf' "${apply_root}/etc/vsftpd.conf" ||
    fail "按用户配置目录未安装"
grep -Fqx 'listen_port=21' "${apply_root}/etc/vsftpd.conf" || fail "FTP 监听端口漂移"
grep -Fqx 'guest_enable=NO' "${apply_root}/etc/vsftpd.conf" || fail "guest_enable 未封闭"
grep -Fqx 'pam_service_name=vsftpd' "${apply_root}/etc/vsftpd.conf" ||
    fail "PAM 服务名漂移"
grep -Fqx 'ssl_enable=NO' "${apply_root}/etc/vsftpd.conf" || fail "SSL 模式未明确"
grep -Fqx 'file_open_mode=0644' "${apply_root}/etc/vsftpd.conf" ||
    fail "全局文件模式未保持组只读"
grep -Fqx 'delete_failed_uploads=YES' "${apply_root}/etc/vsftpd.conf" ||
    fail "失败上传清理未启用"
grep -Fqx 'pasv_address=ftp.example.test' "${apply_root}/etc/vsftpd.conf" ||
    fail "PASV 公网回址未安装"
grep -Fqx 'pasv_addr_resolve=YES' "${apply_root}/etc/vsftpd.conf" ||
    fail "PASV 主机解析未启用"
[[ ! -e "${apply_root}/etc/vsftpd_user_conf/devicedata" ]] ||
    fail "devicedata 未保持 full"
if [[ "$supports_posix_modes" -eq 1 ]]; then
    [[ "$(stat -c '%a' -- "${apply_root}/srv/devicedata")" == "755" ]] ||
        fail "chroot 根不是 root-owned non-writable 0755"
    [[ "$(stat -c '%a' -- "${apply_root}/srv/devicedata/data")" == "2771" ]] ||
        fail "数据根目录不是 2771/traverse-only"
fi
[[ -f "${apply_root}/etc/systemd/system/ota-admin.service" ]] || fail "缺少 ota-admin unit"
grep -Fqx '        allow 127.0.0.1;' "${apply_root}/etc/nginx/sites-available/ota" ||
    fail "admin 未限制 IPv4 loopback"
grep -Fqx '        allow ::1;' "${apply_root}/etc/nginx/sites-available/ota" ||
    fail "admin 未限制 IPv6 loopback"
grep -Fqx '        deny all;' "${apply_root}/etc/nginx/sites-available/ota" ||
    fail "admin 未拒绝公网来源"
grep -Fqx '        proxy_read_timeout 15s;' "${apply_root}/etc/nginx/sites-available/ota" ||
    fail "nginx 管理代理超时未保持 15s"
[[ "$(grep -Fxc '    location ~ ^/ota/(neutral|brand)/latest(?:-v3)?\.json$ {' \
    "${apply_root}/etc/nginx/sites-available/ota")" -eq 1 ]] ||
    fail "nginx 必须用一个精确 location 同时放行 latest.json/latest-v3.json"
NGINX_OTA_CONFIG="${apply_root}/etc/nginx/sites-available/ota" python3 -I - <<'PY' ||
import os
import re
from pathlib import Path

text = Path(os.environ["NGINX_OTA_CONFIG"]).read_text(encoding="utf-8")
line = next(
    (item.strip() for item in text.splitlines() if item.strip().startswith("location ~ ^/ota/")),
    "",
)
pattern = line.removeprefix("location ~ ").removesuffix(" {")
compiled = re.compile(pattern)
allowed = (
    "/ota/neutral/latest.json",
    "/ota/neutral/latest-v3.json",
    "/ota/brand/latest.json",
    "/ota/brand/latest-v3.json",
)
for path in allowed:
    if compiled.fullmatch(path) is None:
        raise SystemExit(f"current/legacy manifest endpoint is not served: {path}")
for path in (
    "/ota/neutral/latest-v4.json",
    "/ota/neutral/payload_history.json",
    "/ota/other/latest-v3.json",
    "/ota/brand/latest-v3.json/extra",
):
    if compiled.fullmatch(path) is not None:
        raise SystemExit(f"nginx manifest location is over-broad: {path}")
PY
    fail "nginx OTA manifest endpoint exact-match regression"
cmp -s -- "$REPO_ROOT/scripts/server/ota_admin.py" "${apply_root}/opt/ota-admin/ota_admin.py" ||
    fail "ota_admin.py 安装内容不一致"
[[ -f "${apply_root}/etc/cron.daily/clean-devicedata" ]] || fail "缺少清理任务"
printf '%s' 'abc' > "${cleanup_root}/scan_20260712T120000123_a1b2c3d4e5f6_4.zip"
printf '%s' 'abc' > "${cleanup_root}/scan_20260712T120001123_a1b2c3d4e5f7_3.zip"
printf '%s' 'abc' > "${cleanup_root}/scan_20260712T120002123_a1b2c3d4e5f8_4.zip"
printf '%s' 'legacy' > "${cleanup_root}/legacy.zip"
mkdir -p -- "${cleanup_root}/empty-device" "${cleanup_root}/device-with-empty-child/empty-child"
touch -d '20 minutes ago' -- \
    "${cleanup_root}/scan_20260712T120000123_a1b2c3d4e5f6_4.zip" \
    "${cleanup_root}/scan_20260712T120001123_a1b2c3d4e5f7_3.zip"
touch -d '31 days ago' -- "${cleanup_root}/legacy.zip"
NO_TEACHING_DATA_ROOT="$cleanup_root" sh "${apply_root}/etc/cron.daily/clean-devicedata"
[[ ! -e "${cleanup_root}/scan_20260712T120000123_a1b2c3d4e5f6_4.zip" ]] ||
    fail "静置的声明字节不匹配残件未删除"
[[ -e "${cleanup_root}/scan_20260712T120001123_a1b2c3d4e5f7_3.zip" ]] ||
    fail "声明字节匹配文件被误删"
[[ -e "${cleanup_root}/scan_20260712T120002123_a1b2c3d4e5f8_4.zip" ]] ||
    fail "未静置 10 分钟的新上传被误删"
[[ ! -e "${cleanup_root}/legacy.zip" ]] || fail "旧命名 30 天清理策略失效"
[[ -d "${cleanup_root}/empty-device" && -d "${cleanup_root}/device-with-empty-child" ]] ||
    fail "清理任务删除了 depth=1 设备账号根目录"
[[ ! -e "${cleanup_root}/device-with-empty-child/empty-child" ]] ||
    fail "清理任务未删除 depth>=2 空子目录"
[[ -s "${apply_root}/opt/ota-admin/token" ]] || fail "管理令牌未生成"
if [[ "$supports_posix_modes" -eq 1 ]]; then
    [[ "$(stat -c '%a' -- "${apply_root}/opt/ota-admin/token")" == "600" ]] ||
        fail "管理令牌权限不是 0600"
fi

apply_with_nss_fixture() {
    DEPLOY_OFFLINE_TEST=1 DEPLOY_ROOT="$apply_root" DEPLOY_CAPACITY_FIXTURE="$capacity_fixture" \
        DEPLOY_NSS_FIXTURE_DIR="$nss_fixture" \
        DEPLOY_OFFLINE_TEST_RELAX_SECRET_MODE="$((1 - supports_posix_modes))" \
        bash "$DEPLOY_SCRIPT" --apply --yes --skip-packages \
        --enable-ufw --ftp-allow-cidr 10.20.0.0/16 \
        --data-filesystem-max-bytes 1073741824 \
        --pasv-address ftp.example.test
}

cat > "${nss_fixture}/passwd" <<'EOF'
devicedata:x:1000:2000::/srv/devicedata:/usr/sbin/nologin
uploader:x:1001:1001::/srv/devicedata:/usr/sbin/nologin
legacy-camera:x:1002:1002::/srv/devicedata:/usr/sbin/nologin
rogue:x:1003:1003::/srv/devicedata:/usr/sbin/nologin
EOF
cat > "${nss_fixture}/group" <<'EOF'
ftpdata:x:995:devicedata,legacy-camera,rogue
devicedata:x:2000:
uploader:x:1001:
legacy-camera:x:1002:
rogue:x:1003:
EOF
mkdir -p -- "${apply_root}/srv/devicedata/data/legacy-camera"
chmod 2770 -- "${apply_root}/srv/devicedata/data/legacy-camera" 2>/dev/null || true
cat > "${apply_root}/etc/vsftpd_user_conf/legacy-camera" <<'EOF'
download_enable=NO
chmod_enable=NO
file_open_mode=0440
local_umask=007
cmds_denied=DELE,RMD,RNFR,RNTO,APPE,REST
local_root=/srv/devicedata
EOF
set +e
rogue_supplementary_output="$(apply_with_nss_fixture 2>&1)"
rogue_supplementary_rc=$?
set -e
[[ $rogue_supplementary_rc -ne 0 ]] || fail "隐藏 supplementary ftpdata 成员未被拒绝"
assert_contains "$rogue_supplementary_output" "rogue 未在最终 vsftpd.userlist"

sed -i 's|local_root=/srv/devicedata|local_root=/srv/devicedata/data/other|' \
    "${apply_root}/etc/vsftpd_user_conf/legacy-camera"
sed -i 's/,rogue//' "${nss_fixture}/group"
set +e
weak_permission_output="$(apply_with_nss_fixture 2>&1)"
weak_permission_rc=$?
set -e
[[ $weak_permission_rc -ne 0 ]] || fail "跨设备 local_root 未被拒绝"
assert_contains "$weak_permission_output" "local_root 必须保持 root-owned"
sed -i 's|local_root=/srv/devicedata/data/other|local_root=/srv/devicedata|' \
    "${apply_root}/etc/vsftpd_user_conf/legacy-camera"
managed_members_output="$(apply_with_nss_fixture)"
assert_contains "$managed_members_output" "部署和本机验证完成"

managed_fingerprint "$apply_root" > "$managed_before"

# 部署器只给本轮新建账号设置初始密码；既有账号轮换必须走 ota-admin 单账号事务。
set +e
existing_password_output="$(apply_once 1 2>&1)"
existing_password_rc=$?
set -e
[[ $existing_password_rc -ne 0 ]] || fail "既有账号密码文件未被拒绝"
assert_contains "$existing_password_output" "既有 devicedata 禁止由部署器改密"
managed_fingerprint "$apply_root" > "$managed_after"
diff -u -- "$managed_before" "$managed_after" >/dev/null ||
    fail "拒绝既有账号改密时产生状态变化"

second_apply_output="$(apply_once 0)"
managed_fingerprint "$apply_root" > "$managed_after"
diff -u -- "$managed_before" "$managed_after" >/dev/null || fail "第二次 apply 产生托管状态漂移"
assert_contains "$second_apply_output" "部署和本机验证完成"

host_accounts_after="$(
    { id devicedata 2>&1 || true; id uploader 2>&1 || true; }
)"
[[ "$host_accounts_before" == "$host_accounts_after" ]] || fail "离线 apply 改动了宿主机账号"

# 真实账号路径无法在无 root 的离线夹具执行，以静态门禁锁住身份校验和回滚顺序。
grep -Fq 'validate_existing_ftp_account devicedata' "$DEPLOY_SCRIPT" ||
    fail "缺少 devicedata 既有账号身份校验"
if grep -Fq 'create_ftp_account uploader' "$DEPLOY_SCRIPT"; then
    fail "部署器仍会创建已退役共享 uploader"
fi
grep -Fq '[[ "$home" == "/srv/devicedata" ]]' "$DEPLOY_SCRIPT" ||
    fail "既有账号 home 门禁漂移"
grep -Fq '[[ "$shell" == "/usr/sbin/nologin" ]]' "$DEPLOY_SCRIPT" ||
    fail "既有账号 shell 门禁漂移"
grep -Fq '"$uid" -ge 1000' "$DEPLOY_SCRIPT" ||
    fail "既有账号 UID0/系统 UID 门禁缺失"
grep -Fq 'UID=${uid} 被其他用户名共享或无法唯一确认' "$DEPLOY_SCRIPT" ||
    fail "既有账号 UID 唯一性门禁缺失"
grep -Fq 'validate_ftpdata_group' "$DEPLOY_SCRIPT" ||
    fail "ftpdata GID 碰撞门禁缺失"
grep -Fq '"$gid" -gt 0' "$DEPLOY_SCRIPT" ||
    fail "ftpdata GID0 门禁缺失"
grep -Fq '被其他组别名占用或无法唯一确认' "$DEPLOY_SCRIPT" ||
    fail "ftpdata 同 GID 别名未 fail closed"
[[ "$(grep -Fc '    validate_ftpdata_membership_closure' "$DEPLOY_SCRIPT")" -ge 1 ]] ||
    fail "ftpdata 成员闭包校验未在部署路径调用"
[[ "$(grep -Fc '    validate_final_ftp_account_state' "$DEPLOY_SCRIPT")" -ge 1 ]] ||
    fail "最终白名单深度账号状态校验缺失"
grep -Fq 'timeout --signal=TERM --kill-after=5s 30s usermod -a -G ftpdata' "$DEPLOY_SCRIPT" ||
    fail "usermod 超时后状态复核门禁漂移"
grep -Fq 'ADDED_DEVICEDATA_TO_FTPDATA=1' "$DEPLOY_SCRIPT" ||
    fail "devicedata 新增组关系未登记"
grep -Fq '} | timeout --signal=TERM --kill-after=5s 10s chpasswd' "$DEPLOY_SCRIPT" ||
    fail "新 devicedata 账号 chpasswd 缺少 10s 阶段预算"
grep -Fq 'create_ftpdata_group' "$DEPLOY_SCRIPT" || fail "groupadd 不确定结果封装缺失"
grep -Fq 'create_ftp_account devicedata' "$DEPLOY_SCRIPT" ||
    fail "devicedata useradd 不确定结果封装缺失"
grep -Fq '严重: 回滚未完整完成' "$DEPLOY_SCRIPT" ||
    fail "回滚失败仍可能被宣称完成"
extract_dependency_loop() {
    local occurrence="$1"
    awk -v wanted="$occurrence" '
        /^[[:space:]]*for command_name in / {
            seen++
            if (seen == wanted) capture = 1
        }
        capture { print }
        capture && /^[[:space:]]*done[[:space:]]*$/ { exit }
    ' "$DEPLOY_SCRIPT"
}

common_dependency_block="$(extract_dependency_loop 1)"
system_dependency_block="$(extract_dependency_loop 2)"
for command_name in awk basename cat chmod chown cp date dirname find grep install mkdir \
    mktemp mv python3 readlink rm sh sha256sum stat tail tr; do
    grep -Eq "(^|[[:space:]\\\\])${command_name}([[:space:];\\\\]|$)" \
        <<< "$common_dependency_block" ||
        fail "${command_name} 未纳入通用依赖命令预检"
done
for command_name in chpasswd findmnt flock getent gpasswd groupadd groupdel id nginx \
    systemctl timeout useradd userdel usermod vsftpd; do
    grep -Eq "(^|[[:space:]\\\\])${command_name}([[:space:];\\\\]|$)" \
        <<< "$system_dependency_block" ||
        fail "${command_name} 未纳入真实模式依赖命令预检"
done
grep -Fq 'validate_ssh_listener_before_ufw_enable' "$DEPLOY_SCRIPT" ||
    fail "UFW enable 缺少 SSH listener 门禁"
grep -Fq '要求 SSH_CONNECTION' "$DEPLOY_SCRIPT" ||
    fail "UFW enable 未绑定当前 SSH 会话端口"
grep -Fq 'command -v ufw' "$DEPLOY_SCRIPT" || fail "UFW 命令未按需预检"
grep -Fq 'command -v ss' "$DEPLOY_SCRIPT" || fail "ss 命令未按需预检"
grep -Fq 'ufw allow from "$FTP_ALLOW_CIDR" to any port 21 proto tcp' "$DEPLOY_SCRIPT" ||
    fail "FTP 主端口未限制到受信 CIDR"
grep -Fq 'ufw allow from "$FTP_ALLOW_CIDR" to any port 40000:40100 proto tcp' "$DEPLOY_SCRIPT" ||
    fail "FTP 被动端口未限制到受信 CIDR"
if grep -Fq 'ufw allow 21/tcp' "$DEPLOY_SCRIPT"; then
    fail "FTP 主端口仍默认向公网放行"
fi
grep -Fq 'LC_ALL=C ufw show added' "$DEPLOY_SCRIPT" ||
    fail "配置 FTP 防火墙前未审计既有 UFW 规则"
grep -Fq '发现更宽或无法证明等价的旧 FTP UFW 规则' "$DEPLOY_SCRIPT" ||
    fail "更宽旧 FTP UFW 规则未 fail closed"
grep -Fq 'verify_final_private_ftp_firewall' "$DEPLOY_SCRIPT" ||
    fail "最终 UFW active/CIDR 远端回读缺失"
grep -Fq 'DEFAULT_INPUT_POLICY="DROP"' "$DEPLOY_SCRIPT" ||
    fail "UFW 默认入站 deny 前置门禁缺失"
grep -Fq 'Default: deny \(incoming\)' "$DEPLOY_SCRIPT" ||
    fail "UFW 最终默认入站 deny 回读缺失"
grep -Fq 'validate_capacity_boundary' "$DEPLOY_SCRIPT" ||
    fail "独立受限文件系统容量硬门禁缺失"
grep -Fq 'atomic_install_file "$OTA_ADMIN_STAGED" "$OTA_ADMIN_TARGET"' "$DEPLOY_SCRIPT" ||
    fail "ota_admin.py 未从已校验 staged bytes 安装"
grep -Fq 'ota_admin.py 安装后哈希不一致' "$DEPLOY_SCRIPT" ||
    fail "ota_admin.py 缺少安装后回读哈希"
if ! awk '
    /cp --no-dereference -- "\$OTA_ADMIN_SOURCE" "\$OTA_ADMIN_STAGED"/ { staged = 1; next }
    staged && /\$OTA_ADMIN_SOURCE/ { exit 1 }
    END { exit(staged ? 0 : 1) }
' "$DEPLOY_SCRIPT"; then
    fail "staging 后仍重新解引用 OTA_ADMIN_SOURCE"
fi
grep -Fq 'validate_root_only_parent_chain "$secret_source" "秘密文件"' "$DEPLOY_SCRIPT" ||
    fail "秘密文件父目录 TOCTOU 门禁缺失"
grep -Fq 'readonly REAL_STAGE_BASE="/var/tmp/no-teaching-online-services"' "$DEPLOY_SCRIPT" ||
    fail "真实 apply 未使用固定 root staging 根目录"
grep -Fq 'STAGE_DIR="$(mktemp -d "${REAL_STAGE_BASE}/stage.XXXXXX")"' "$DEPLOY_SCRIPT" ||
    fail "真实 apply staging 仍可能受 TMPDIR 劫持"

lock_line="$(grep -nF '    acquire_account_mutation_lock' "$DEPLOY_SCRIPT" | tail -n1 | cut -d: -f1)"
stop_admin_line="$(grep -nF '        systemctl stop ota-admin' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
revalidate_lock_line="$(grep -nF '    revalidate_account_lock_after_admin_stop' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
refresh_line="$(grep -nF '    refresh_ftp_candidates' "$DEPLOY_SCRIPT" | tail -n1 | cut -d: -f1)"
release_line="$(grep -nF '    release_account_mutation_lock' "$DEPLOY_SCRIPT" | tail -n1 | cut -d: -f1)"
restart_admin_line="$(grep -nF '    systemctl restart vsftpd ota-admin' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
commit_line="$(grep -nF 'TRANSACTION_COMMITTED=1' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
[[ "$lock_line" -lt "$stop_admin_line" && "$stop_admin_line" -lt "$revalidate_lock_line" &&
   "$revalidate_lock_line" -lt "$refresh_line" &&
   "$refresh_line" -lt "$restart_admin_line" && "$restart_admin_line" -lt "$commit_line" &&
   "$commit_line" -lt "$release_line" ]] ||
    fail "共享账号锁未覆盖 stop/re-read/install/restart 全窗口"
grep -Fq 'readonly ACCOUNT_LOCK_DIR="/run/no-teaching-ota"' "$DEPLOY_SCRIPT" ||
    fail "账号事务锁仍位于共享 /run/lock 根"
grep -Fq 'readonly ACCOUNT_LOCK_FILE="${ACCOUNT_LOCK_DIR}/ota-accounts.lock"' "$DEPLOY_SCRIPT" ||
    fail "账号事务锁文件路径漂移"
grep -Fq 'install -d -m 0700 -o root -g root -- "$ACCOUNT_LOCK_DIR"' "$DEPLOY_SCRIPT" ||
    fail "部署器未安全创建 root-only 账号锁目录"
grep -Fq 'account_lock_directory_is_secure' "$DEPLOY_SCRIPT" ||
    fail "账号锁目录缺少 root:root 0700 回读门禁"
grep -Fq 'stat -Lc '\''%d:%i'\'' -- "/proc/$$/fd/${ACCOUNT_LOCK_FD}"' "$DEPLOY_SCRIPT" ||
    fail "账号锁未比较路径与持锁 FD inode"
if grep -Fq '/run/lock/no-teaching-ota-accounts.lock' "$DEPLOY_SCRIPT"; then
    fail "部署器仍引用本地用户可抢占的旧共享锁"
fi

rollback_stop_line="$(grep -nF 'systemctl stop "${SERVICE_NAMES[$i]}"' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
rollback_start_line="$(grep -nF 'systemctl start "$svc"' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
rollback_account_line="$(grep -nF 'userdel devicedata' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
[[ "$rollback_stop_line" -lt "$rollback_account_line" && "$rollback_account_line" -lt "$rollback_start_line" ]] ||
    fail "rollback 服务恢复发生在账号/目录恢复之前"
runtime_branch_line="$(grep -nF '                enabled-runtime)' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
runtime_disable_line="$(grep -nF '                        systemctl disable "$svc"' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
runtime_enable_line="$(grep -nF '                        systemctl enable --runtime "$svc"' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
runtime_verify_line="$(grep -nF '== "enabled-runtime"' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
[[ "$runtime_branch_line" -lt "$runtime_disable_line" &&
   "$runtime_disable_line" -lt "$runtime_enable_line" &&
   "$runtime_enable_line" -lt "$runtime_verify_line" ]] ||
    fail "rollback 未先清除持久 enable 再精确恢复 enabled-runtime"
grep -Fq '                enabled)' "$DEPLOY_SCRIPT" ||
    fail "rollback 缺少持久 enabled 独立分支"

if grep -Eq '^[[:space:]]*(sudo[[:space:]]+)?apt(-get)?([[:space:]]|$)' "$DEPLOY_SCRIPT"; then
    fail "部署器仍会自动安装/升级软件包并触发 vsftpd postinst"
fi
[[ "$(head -n1 "$DEPLOY_SCRIPT")" == '#!/bin/bash' ]] ||
    fail "部署器 shebang 仍可能通过调用方 PATH 解析 bash"
trace_off_line="$(grep -nF 'set +x +v' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
trusted_path_line="$(grep -nF 'export PATH="/usr/sbin:/usr/bin:/sbin:/bin"' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
first_external_line="$(grep -nF '/usr/bin/readlink -f -- "$DEPLOY_ROOT"' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
[[ "$trace_off_line" -lt "$trusted_path_line" && "$trusted_path_line" -lt "$first_external_line" ]] ||
    fail "xtrace/可信 PATH 门禁未先于首个外部命令"
if grep -Eq 'python3[[:space:]]+-([[:space:]]|$)' "$DEPLOY_SCRIPT"; then
    fail "存在未使用 isolated mode 的 inline Python"
fi
grep -Fq 'ExecStart=/usr/bin/python3 -I /opt/ota-admin/ota_admin.py' "$DEPLOY_SCRIPT" ||
    fail "ota-admin systemd Python 未启用 isolated mode"
grep -Fq '.no-teaching-online-services-offline-root' "$DEPLOY_SCRIPT" ||
    fail "离线 DEPLOY_ROOT 缺少 marker 门禁"
grep -Fq 'systemctl mask --runtime --now vsftpd.service' "$DEPLOY_SCRIPT" ||
    fail "帮助文本缺少预装维护窗口的 vsftpd runtime-mask 步骤"
grep -Fq 'systemctl unmask --runtime vsftpd.service' "$DEPLOY_SCRIPT" ||
    fail "帮助文本可能误解除既有 persistent vsftpd mask"
apply_ack_line="$(grep -nF 'if [[ "$MODE" == "apply" && "$PACKAGES_PREINSTALLED_ACK" -ne 1 ]]' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
preinstall_line="$(grep -nF '验证外部维护窗口已预装依赖' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
stage_create_line="$(grep -nF 'STAGE_DIR="$(mktemp -d' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
stop_vsftpd_line="$(grep -nF '        systemctl stop vsftpd' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
final_refresh_line="$(grep -nF '    refresh_ftp_candidates' "$DEPLOY_SCRIPT" | tail -n1 | cut -d: -f1)"
install_userlist_line="$(grep -nF 'atomic_install_file "${STAGE_DIR}/vsftpd.userlist"' "$DEPLOY_SCRIPT" | head -n1 | cut -d: -f1)"
[[ "$apply_ack_line" -lt "$preinstall_line" && "$preinstall_line" -lt "$stage_create_line" &&
   "$stage_create_line" -lt "$stop_vsftpd_line" && "$stop_vsftpd_line" -lt "$final_refresh_line" &&
   "$final_refresh_line" -lt "$install_userlist_line" ]] ||
    fail "apply 确认/依赖验证未先于 staging，或停服/刷新候选/原子安装排序错误"

echo "deploy_online_services 离线门禁通过"
