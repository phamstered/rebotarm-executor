import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  title: 'reBot Arm Executor',
  description:
    'STM32H723 FreeRTOS micro-ROS 电机桥接固件 — Damiao MIT 控制、FDCAN、USB CDC',
  base: '/rebotarm-executor/',
  cleanUrls: true,
  ignoreDeadLinks: true,
  themeConfig: {
    nav: [
      { text: '介绍', link: '/' },
      { text: '架构', link: '/architecture' },
      { text: '构建', link: '/build' },
      { text: 'ROS 2 接口', link: '/ros2-interface' },
      {
        text: 'GitHub',
        link: 'https://github.com/phamstered/rebotarm-executor',
      },
    ],
    sidebar: [
      {
        text: '开始使用',
        items: [
          { text: '项目介绍', link: '/' },
          { text: '架构概览', link: '/architecture' },
          { text: '构建与烧录', link: '/build' },
        ],
      },
      {
        text: '接口与集成',
        items: [
          { text: 'ROS 2 接口', link: '/ros2-interface' },
          { text: 'FDCAN 集成', link: '/fdcan-integration' },
          { text: 'micro-ROS Session 稳定性', link: '/microros-session-stability' },
          { text: 'Vofa+ USB CDC', link: '/vofa-usb-cdc-integration' },
        ],
      },
    ],
    socialLinks: [
      {
        icon: 'github',
        link: 'https://github.com/phamstered/rebotarm-executor',
      },
    ],
    outline: { level: [2, 3] },
    search: { provider: 'local' },
    footer: {
      message: 'User docs for reBot Arm Executor (rebotarm-executor)',
      copyright: 'Source: AGENTS.md, PLAN.md, CLAUDE.md and docs/ in the repository',
    },
  },
})
