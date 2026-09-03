# Git 日常操作文档

本文适用于以下仓库和分支：

- 本地目录：`D:\CraftWork\NoTeaching-Robot-底层优化适配`
- GitHub 仓库：`https://github.com/yu1201/NoTeaching-Robot.git`
- 工作分支：`底层优化适配`
- 整理日期：`2026-09-01`

目标是安全完成四类日常操作：

1. 检查 GitHub 是否有更新。
2. 把远端更新拉到本地。
3. 把本地代码修改提交并上传到 GitHub。
4. 在本地和远端同时有修改时避免覆盖代码。

## 一、必须先理解的概念

| 名称 | 含义 |
| --- | --- |
| 工作区 | 当前磁盘上的文件，可以包含尚未提交的修改。 |
| 暂存区 | 通过 `git add` 选中、准备放进下一次提交的文件。 |
| 本地提交 | 已执行 `git commit`，但不一定上传到 GitHub。 |
| `origin` | 本项目的 GitHub 远端名称。 |
| `HEAD` | 当前本地分支最新提交。 |
| `origin/底层优化适配` | 最近一次 `fetch` 后，本机记录的 GitHub 分支位置。 |
| `fetch` | 获取远端提交信息，不修改当前工作区。 |
| `pull` | 获取远端提交并更新当前分支。 |
| `push` | 把本地提交上传到 GitHub。 |
| `stash` | 临时保存尚未提交的已跟踪文件修改。 |

推荐始终使用 `--ff-only` 拉取。它只允许安全快进；如果本地和远端已分叉，会拒绝操作，而不是自动制造难以检查的合并提交。

## 二、打开仓库并确认分支

在 PowerShell 中执行：

```powershell
cd D:\CraftWork\NoTeaching-Robot-底层优化适配
git branch --show-current
```

正确输出应为：

```text
底层优化适配
```

如果不是该分支，并且当前没有未提交修改，可切换：

```powershell
git switch 底层优化适配
```

也可以不切换目录，使用 `-C`：

```powershell
git -C D:\CraftWork\NoTeaching-Robot-底层优化适配 status
```

## 三、查看本地状态

```powershell
git status --short --branch
```

常见状态标记：

| 标记 | 含义 |
| --- | --- |
| ` M file.cpp` | 已跟踪文件在工作区被修改，尚未暂存。 |
| `M  file.cpp` | 修改已经暂存。 |
| `MM file.cpp` | 文件暂存后又继续被修改。 |
| `A  file.cpp` | 新文件已经暂存。 |
| ` D file.cpp` | 已跟踪文件在工作区被删除。 |
| `?? file` | 未跟踪的新文件或目录。 |
| `UU file.cpp` | 文件存在合并冲突。 |

本项目经常出现以下运行产物：

```text
Job/Inovance/
Job/STEP/endpoint_*/
```

其中通常包含 `.srp`、`.srd`、`.pro`、`.prj` 文件。这些是机器人程序或运行期产物，不应因为执行 `git add .` 被误上传。

## 四、检查 GitHub 是否有更新

### 1. 获取远端信息

```powershell
git fetch origin
```

`fetch` 不会改变当前代码，可以安全执行。

网络连接不稳定时，可以使用：

```powershell
git -c http.version=HTTP/1.1 fetch origin
```

### 2. 比较本地和远端

```powershell
git rev-list --left-right --count HEAD...origin/底层优化适配
```

结果解释：

| 输出 | 含义 | 处理方式 |
| --- | --- | --- |
| `0  0` | 本地和远端一致。 | 不需要拉取或上传。 |
| `0  2` | 本地落后远端 2 个提交。 | 检查后执行拉取。 |
| `1  0` | 本地领先远端 1 个提交。 | 本地提交尚未上传。 |
| `1  2` | 本地与远端都有独立提交。 | 已分叉，不要直接推送或普通拉取。 |

查看 GitHub 比本地多出的提交：

```powershell
git log --oneline HEAD..origin/底层优化适配
```

查看本地尚未上传的提交：

```powershell
git log --oneline origin/底层优化适配..HEAD
```

查看双方完整提交号：

```powershell
git rev-parse HEAD
git rev-parse origin/底层优化适配
```

如果怀疑更新被推到了其他分支：

```powershell
git ls-remote --heads origin
```

## 五、拉取 GitHub 更新

### 场景 A：没有已跟踪文件修改

先检查：

```powershell
git status --short --branch
```

如果只有 `?? Job/...` 运行产物，没有 `M`、`A`、`D` 等已跟踪修改，可以执行：

```powershell
git pull --ff-only origin 底层优化适配
```

### 场景 B：存在尚未提交的已跟踪修改

先临时保存修改：

```powershell
git stash push -m "拉取前临时保存"
```

再拉取：

```powershell
git pull --ff-only origin 底层优化适配
```

恢复本地修改：

```powershell
git stash pop
```

完整流程：

```powershell
git status --short --branch
git fetch origin
git stash push -m "拉取前临时保存"
git pull --ff-only origin 底层优化适配
git stash pop
git status --short --branch
```

默认的 `git stash push` 只保存已跟踪文件修改，不会把 `?? Job/STEP/...` 临时目录放进 stash。不要随意添加 `-u`，否则会把大量未跟踪运行产物也收进去。

### 拉取后的核验

```powershell
git fetch origin
git rev-list --left-right --count HEAD...origin/底层优化适配
```

正确结果应为：

```text
0    0
```

## 六、上传本地代码修改

上传必须经过以下顺序：

```text
检查差异 → 获取远端更新 → 选择文件暂存 → 检查暂存内容 → 提交 → 推送 → 核验
```

### 1. 查看修改内容

```powershell
git status --short --branch
git diff --stat
git diff
git diff --check
```

说明：

- `git diff --stat`：查看改了哪些文件以及修改规模。
- `git diff`：查看具体代码差异。
- `git diff --check`：检查多余空格和部分格式问题；正常情况下不输出内容且退出码为 0。

### 2. 提交前再次检查远端

```powershell
git fetch origin
git rev-list --left-right --count HEAD...origin/底层优化适配
```

如果结果是 `0  N`，先按第五节拉取远端更新，再提交本地代码。

### 3. 选择要提交的文件

#### 只提交已经被 Git 管理的修改文件

```powershell
git add -u
```

这会暂存已跟踪文件的修改和删除，不会暂存 `??` 未跟踪文件，因此适合本项目日常使用。

#### 新增了真正的源代码文件

必须显式添加具体文件：

```powershell
git add src/NewFeature.cpp
git add include/NewFeature.h
git add scripts/tests/verify_new_feature.py
```

不要在本项目根目录直接执行：

```powershell
git add .
```

否则可能把 `Job/Inovance`、`Job/STEP/endpoint_*` 等运行产物一起放入提交。

### 4. 检查即将提交的内容

```powershell
git diff --cached --name-status
git diff --cached --stat
git diff --cached
git diff --cached --check
```

特别检查暂存列表中是否出现：

```text
Job/Inovance/
Job/STEP/endpoint_*/
Data/
Result/
*.exe
*.dll
*.zip
```

如果误暂存了文件，可以取消暂存，但保留磁盘文件：

```powershell
git restore --staged 路径
```

例如：

```powershell
git restore --staged Job/Inovance
```

### 5. 运行相关测试

根据修改范围运行对应测试。例如扫描变姿态相关修改：

```powershell
py -3 scripts/tests/verify_scan_pose_variation_test.py
py -3 scripts/tests/verify_pointcloud_quality_gate.py
py -3 scripts/tests/verify_weld_safe_retreat_terminal.py
```

测试失败时不要上传，应先修复或明确记录失败原因。

### 6. 创建本地提交

```powershell
git commit -m "feat: 简短描述本次功能"
```

常用提交类型：

| 类型 | 用途 | 示例 |
| --- | --- | --- |
| `feat` | 新功能 | `feat: 增加扫描直线模拟流程` |
| `fix` | 修复问题 | `fix: 修复扫描收枪状态判断` |
| `refactor` | 重构但不改变功能 | `refactor: 整理机器人驱动接口` |
| `test` | 测试相关 | `test: 补充轨迹安全门禁验证` |
| `docs` | 仅文档 | `docs: 增加 Git 操作说明` |
| `build` | 构建、工程配置 | `build: 更新 Visual Studio 工程文件` |

查看刚创建的提交：

```powershell
git log -1 --oneline --stat
```

### 7. 上传到 GitHub

```powershell
git push origin HEAD:底层优化适配
```

成功输出类似：

```text
69d92a9..456a0a4  HEAD -> 底层优化适配
```

### 8. 上传后的核验

```powershell
git fetch origin
git rev-list --left-right --count HEAD...origin/底层优化适配
git status --short --branch
```

应满足：

- `rev-list` 输出 `0  0`。
- 没有已跟踪文件修改。
- 如果仍显示 `?? Job/...`，只表示本地还有未跟踪运行产物，不表示代码没有上传。

## 七、本地和远端同时有提交

如果执行：

```powershell
git rev-list --left-right --count HEAD...origin/底层优化适配
```

得到类似：

```text
1    2
```

说明分支已分叉。不要执行强制推送，也不要随意使用普通 `git pull`。

确认工作区干净后，可以把本地提交重新放到远端最新提交之后：

```powershell
git rebase origin/底层优化适配
```

如果发生冲突：

1. 执行 `git status` 查看冲突文件。
2. 手工编辑冲突文件，删除 `<<<<<<<`、`=======`、`>>>>>>>` 标记并保留正确内容。
3. 暂存已解决文件：

   ```powershell
   git add 冲突文件
   ```

4. 继续 rebase：

   ```powershell
   git rebase --continue
   ```

5. 如果判断无法安全解决，取消本次 rebase：

   ```powershell
   git rebase --abort
   ```

解决并测试后再推送：

```powershell
git push origin HEAD:底层优化适配
```

默认禁止使用：

```powershell
git push --force
```

它可能覆盖其他人的远端提交。即使是 `--force-with-lease`，也只应在团队明确同意改写历史时使用。

## 八、常见问题处理

### 1. GitHub 返回 403

典型信息：

```text
Permission to yu1201/NoTeaching-Robot.git denied
The requested URL returned error: 403
```

原因通常是当前 GitHub 账号没有写权限，或尚未接受协作者邀请。

查看 Git Credential Manager：

```powershell
git credential-manager --version
git credential-manager github list
```

使用有权限的账号重新登录：

```powershell
git credential-manager github login --username 账号名 --browser --force
```

完成网页登录后重新执行 `git push`。不要把 Personal Access Token 写入脚本、文档或远端 URL。

### 2. GitHub 连接重置或超时

先重试：

```powershell
git -c http.version=HTTP/1.1 fetch origin
git -c http.version=HTTP/1.1 push origin HEAD:底层优化适配
```

不要因为一次网络失败就判断“没有更新”或“上传成功”。必须用提交号和 `0  0` 状态核验。

### 3. `stash pop` 发生冲突

```powershell
git status
```

逐个解决冲突后：

```powershell
git add 冲突文件
```

在确认修改全部恢复前，不要删除 stash。查看 stash：

```powershell
git stash list
git stash show --stat stash@{0}
```

### 4. 提交后发现漏了文件，但尚未上传

```powershell
git add 漏掉的文件
git commit --amend --no-edit
```

如果提交已经上传，通常新建一个修复提交更安全，不要随意改写已公开历史。

### 5. 已上传错误修改

使用反向提交，而不是删除远端历史：

```powershell
git revert 错误提交号
git push origin HEAD:底层优化适配
```

## 九、禁止或慎用的命令

以下命令可能丢失代码或覆盖他人提交：

```powershell
git reset --hard
git clean -fd
git checkout -- 文件
git restore 文件
git push --force
```

除非已经确认目标和后果，否则不要执行。

- `reset --hard` 会丢弃已跟踪文件的本地修改。
- `clean -fd` 会删除未跟踪文件和目录，可能清掉 Job/STEP 产物。
- `restore 文件` 会丢弃指定文件尚未提交的修改。
- `push --force` 可能覆盖 GitHub 上其他人的提交。

## 十、推荐的日常命令清单

### 只检查更新

```powershell
cd D:\CraftWork\NoTeaching-Robot-底层优化适配
git branch --show-current
git status --short --branch
git fetch origin
git rev-list --left-right --count HEAD...origin/底层优化适配
git log --oneline HEAD..origin/底层优化适配
```

### 安全拉取

```powershell
git status --short --branch
git fetch origin
git pull --ff-only origin 底层优化适配
git rev-list --left-right --count HEAD...origin/底层优化适配
```

### 上传已有代码修改

```powershell
git status --short --branch
git diff --stat
git diff --check
git fetch origin
git rev-list --left-right --count HEAD...origin/底层优化适配
git add -u
git diff --cached --name-status
git diff --cached --check
git commit -m "feat: 描述本次修改"
git push origin HEAD:底层优化适配
git fetch origin
git rev-list --left-right --count HEAD...origin/底层优化适配
git status --short --branch
```

### 上传新增源文件

```powershell
git add src/NewFeature.cpp include/NewFeature.h
git diff --cached --name-status
git commit -m "feat: 增加新功能"
git push origin HEAD:底层优化适配
```

## 十一、最终判断标准

一次操作只有同时满足以下条件，才算真正完成：

### 拉取完成

- 当前分支是 `底层优化适配`。
- `HEAD` 与 `origin/底层优化适配` 提交号一致。
- `git rev-list --left-right --count ...` 输出 `0  0`。
- 拉取前的本地修改仍然存在且没有未解决冲突。

### 上传完成

- 需要上传的文件全部出现在目标提交中。
- 相关测试通过。
- `git push` 明确返回成功。
- 本地和远端提交号一致。
- `git rev-list --left-right --count ...` 输出 `0  0`。
- 工作区没有遗漏的已跟踪修改。
- 未跟踪的 Job/STEP 运行产物没有被误上传。
