# 项目文档

这份目录用于把当前 Notion 里的核心项目文档同步成 GitHub 可直接预览的 Markdown。

- 人工整理日期：`2026-07-11`
- Notion 总入口：<https://www.notion.so/336c868d819b80a181c7ef8a393b53ad>

## 当前同步页

- [工作流程](workflow.md)
- [界面说明](ui.md)
- [CLI 命令说明书](cli-commands.md)
- [运行目录与旧数据迁移](runtime-paths.md)
- [本地账号与凭据安全](security-credentials.md)
- [在线服务部署](server-deployment.md)
- [CI 离线门禁](ci.md)
- [代码环境搭建说明书](source-environment-setup.md)
- [第三方依赖离线包说明](third-party-offline-package.md)
- [波纹板项目方案](wavy-board-plan.md)
- [问题表摘要](issues.md)
- [工作记录摘要](worklog.md)

## 维护约定

- Notion 作为协作主文档，记录更完整的页面结构、数据库和现场补充信息。
- GitHub `docs/` 作为代码仓库内的快照文档，重点保留和当前实现强绑定的内容。
- 当代码逻辑、参数约定或流程有明显变化时，建议同步更新 Notion 和这里的 Markdown。
