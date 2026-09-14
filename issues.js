// issues.js
import { state, showNotification } from './core.js';

async function loadIssues() {
    try {
        // 使用 fetch 读取同目录下的 issues.json（Electron 中相对路径基于 index.html）
        const response = await fetch('./issues.json');
        if (!response.ok) throw new Error('加载失败');
        const data = await response.json();
        state.issues = data;
        renderIssueList();
    } catch (error) {
        console.error('读取 issues.json 失败:', error);
        showNotification('无法加载问题列表，请检查 issues.json 文件是否存在', 'error');
        state.issues = [];
        renderIssueList();
    }
}

function renderIssueList() {
    const container = document.getElementById('issueList');
    if (!container) return;
    if (!state.issues.length) {
        container.innerHTML = '<div style="text-align:center; color:#94a3b8; padding:20px;">暂无问题，请手动编辑 issues.json 添加</div>';
        return;
    }
    let html = '';
    state.issues.forEach(issue => {
        const statusColor = issue.status === 'resolved' ? '#10b981' : '#ef4444';
        const statusText = issue.status === 'resolved' ? '已解决' : '未解决';
        const priorityColor = issue.priority === 'high' ? '#ef4444' : issue.priority === 'medium' ? '#f59e0b' : '#3b82f6';

        // 如果有 resolution 且不为空，则显示
        let resolutionHtml = '';
        if (issue.resolution && issue.resolution.trim() !== '') {
            resolutionHtml = `<div style="font-size:12px; color:#10b981; margin-top:4px;">✅ 解决方案：${escapeHtml(issue.resolution)}</div>`;
        }

        html += `
            <div style="background:#f8fafc; border-radius:16px; padding:12px; border-left:4px solid ${priorityColor};">
                <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:8px;">
                    <div style="font-weight:600;">${escapeHtml(issue.title)}</div>
                    <div style="display:flex; gap:8px;">
                        <span style="font-size:11px; background:${statusColor}; color:white; padding:2px 8px; border-radius:20px;">${statusText}</span>
                    </div>
                </div>
                <div style="font-size:12px; color:#475569; margin-bottom:8px;">${escapeHtml(issue.description)}</div>
                ${resolutionHtml}
                <div style="font-size:10px; color:#94a3b8;">优先级: ${issue.priority} | 创建于: ${issue.createdAt}</div>
            </div>
        `;
    });
    container.innerHTML = html;
}

function escapeHtml(str) {
    return str.replace(/[&<>]/g, function(m) {
        if (m === '&') return '&amp;';
        if (m === '<') return '&lt;';
        if (m === '>') return '&gt;';
        return m;
    });
}

export function initIssues() {
    loadIssues();
}