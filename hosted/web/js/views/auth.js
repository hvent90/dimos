// Auth view — login / signup / email verification / password reset.
// All flows talk directly to Cognito (see cognito.js); on success we store
// the ID + refresh tokens and the broker just verifies them per-request.

import { setSession } from '../api.js';
import { navigate } from '../router.js';
import { escHtml, state } from '../state.js';
import {
    confirmForgotPassword,
    confirmSignUp,
    forgotPassword,
    login,
    resendCode,
    signUp,
    tokenPayload,
} from '../cognito.js';

// 'login' | 'register' | 'confirm' | 'forgot' | 'forgot-confirm'
let mode = 'login';
let pendingEmail = '';
let pendingPassword = '';  // kept in-memory only, to auto-login after confirm

export function renderAuth(c) {
    if (mode === 'confirm') return renderConfirm(c);
    if (mode === 'forgot' || mode === 'forgot-confirm') return renderForgot(c);
    renderLoginRegister(c);
}

function shell(inner) {
    return `
    <div class="min-h-screen flex flex-col items-center justify-center p-4">
        <div class="w-full max-w-md fade-in">
            <div class="text-center mb-10 select-none">
                <img src="assets/dimensional-logo.png" alt="DIMENSIONAL" draggable="false"
                     class="crt-glow mx-auto mb-4 w-full max-w-[340px]">
                <p class="term-caps text-dim-500 text-xs">Teleop console — operator access</p>
            </div>
            <div class="bg-bg-950/90 border border-[#2a2a2a] p-6 shadow-xl">
                ${inner}
            </div>
            <div class="term-caps text-center text-gray-600 text-[10px] leading-relaxed mt-8">
                <p>© 2026 Dimensional Inc. All rights reserved.</p>
                <p>This interface is part of the <a href="https://dimensionalos.com" class="text-dim-700 hover:text-dim-500">Dimensional operating system</a>.</p>
                <p>Unauthorized duplication, transmission, or interdimensional transfer is highly prohibited.</p>
            </div>
        </div>
    </div>`;
}

const inputCls = 'w-full px-4 py-2.5 bg-[#0d0e0e] border border-[#2a2a2a] text-white placeholder-gray-600 text-sm focus:outline-none focus:border-dim-500 focus:ring-1 focus:ring-dim-500';
const buttonCls = 'term-caps w-full py-2.5 bg-dim-500 hover:bg-dim-400 text-bg-950 text-sm font-bold transition-colors';

function showError(msg) {
    const el = document.getElementById('auth-error');
    el.textContent = msg;
    el.classList.remove('hidden');
}

// --- Login / Register ---

function renderLoginRegister(c) {
    const isLogin = mode === 'login';
    c.innerHTML = shell(`
        <div class="flex mb-6 border border-[#2a2a2a] p-1 gap-1">
            <button id="tab-login" class="flex-1 py-2 px-4"></button>
            <button id="tab-register" class="flex-1 py-2 px-4"></button>
        </div>
        <form id="auth-form">
            <div class="space-y-4">
                <div>
                    <label class="term-caps block text-xs text-gray-400 mb-1.5">Email</label>
                    <input id="email" type="email" required value="${escHtml(state.userEmail)}"
                        class="${inputCls}" placeholder="you@company.com">
                </div>
                <div>
                    <label class="term-caps block text-xs text-gray-400 mb-1.5">Password</label>
                    <input id="password" type="password" required minlength="8"
                        autocomplete="${isLogin ? 'current-password' : 'new-password'}"
                        class="${inputCls}" placeholder="••••••••">
                    ${isLogin ? '' : '<p class="text-gray-500 text-xs mt-1">At least 8 characters</p>'}
                </div>
                <div id="auth-error" class="text-red-400 text-sm hidden"></div>
                <button type="submit" class="${buttonCls}">
                    <span id="auth-btn-text">${isLogin ? '[ LOG IN ]' : '[ CREATE ACCOUNT ]'}</span>
                </button>
                ${isLogin ? '<button type="button" id="forgot-link" class="term-caps w-full text-xs text-gray-500 hover:text-dim-400">Forgot password?</button>' : ''}
            </div>
        </form>`);

    styleTabs();
    document.getElementById('tab-login').onclick = () => { mode = 'login'; renderAuth(c); };
    document.getElementById('tab-register').onclick = () => { mode = 'register'; renderAuth(c); };
    document.getElementById('auth-form').onsubmit = (e) => handleAuth(e, c);
    const forgot = document.getElementById('forgot-link');
    if (forgot) forgot.onclick = () => {
        pendingEmail = document.getElementById('email').value.trim();
        mode = 'forgot';
        renderAuth(c);
    };
}

function styleTabs() {
    const active = 'term-caps flex-1 py-2 px-4 text-xs font-bold bg-dim-500 text-bg-950';
    const inactive = 'term-caps flex-1 py-2 px-4 text-xs text-gray-500 hover:text-white';
    document.getElementById('tab-login').className = mode === 'login' ? active : inactive;
    document.getElementById('tab-register').className = mode === 'register' ? active : inactive;
    document.getElementById('tab-login').textContent = 'Log In';
    document.getElementById('tab-register').textContent = 'Sign Up';
}

async function handleAuth(e, c) {
    e.preventDefault();
    const email = document.getElementById('email').value.trim();
    const password = document.getElementById('password').value;
    const btn = document.getElementById('auth-btn-text');
    document.getElementById('auth-error').classList.add('hidden');
    btn.textContent = mode === 'login' ? '[ LOGGING IN… ]' : '[ CREATING ACCOUNT… ]';
    try {
        if (mode === 'login') {
            await doLogin(email, password);
        } else {
            const res = await signUp(email, password);
            pendingEmail = email;
            pendingPassword = password;
            if (res.UserConfirmed) {
                await doLogin(email, password);
            } else {
                mode = 'confirm';
                renderAuth(c);
            }
        }
    } catch (err) {
        if (err.message === 'UserNotConfirmedException') {
            // Logged in before verifying email — resume the confirm flow.
            pendingEmail = email;
            pendingPassword = password;
            try { await resendCode(email); } catch { /* throttled is fine */ }
            mode = 'confirm';
            renderAuth(c);
            return;
        }
        btn.textContent = mode === 'login' ? '[ LOG IN ]' : '[ CREATE ACCOUNT ]';
        showError(err.message);
    }
}

async function doLogin(email, password) {
    const result = await login(email, password);
    const claims = tokenPayload(result.IdToken) || {};
    setSession(result.IdToken, result.RefreshToken, claims.email || email);
    pendingPassword = '';
    navigate('dashboard');
}

// --- Email verification ---

function renderConfirm(c) {
    c.innerHTML = shell(`
        <h2 class="term-caps text-sm font-bold text-dim-500 mb-1">// Check your email</h2>
        <p class="text-gray-400 text-sm mb-6">We sent a verification code to
            <span class="text-white">${escHtml(pendingEmail)}</span></p>
        <form id="confirm-form">
            <div class="space-y-4">
                <div>
                    <label class="term-caps block text-xs text-gray-400 mb-1.5">Verification code</label>
                    <input id="code" inputmode="numeric" autocomplete="one-time-code" required
                        class="${inputCls} tracking-[0.5em] text-center" placeholder="······">
                </div>
                <div id="auth-error" class="text-red-400 text-sm hidden"></div>
                <button type="submit" class="${buttonCls}"><span id="auth-btn-text">[ VERIFY ]</span></button>
                <div class="flex justify-between">
                    <button type="button" id="resend" class="term-caps text-xs text-gray-500 hover:text-dim-400">Resend code</button>
                    <button type="button" id="back" class="term-caps text-xs text-gray-500 hover:text-dim-400">Back to login</button>
                </div>
            </div>
        </form>`);

    document.getElementById('back').onclick = () => { mode = 'login'; renderAuth(c); };
    document.getElementById('resend').onclick = async () => {
        try { await resendCode(pendingEmail); showError('Code re-sent'); }
        catch (err) { showError(err.message); }
    };
    document.getElementById('confirm-form').onsubmit = async (e) => {
        e.preventDefault();
        const btn = document.getElementById('auth-btn-text');
        btn.textContent = '[ VERIFYING… ]';
        try {
            await confirmSignUp(pendingEmail, document.getElementById('code').value.trim());
            if (pendingPassword) {
                await doLogin(pendingEmail, pendingPassword);
            } else {
                mode = 'login';
                renderAuth(c);
            }
        } catch (err) {
            btn.textContent = '[ VERIFY ]';
            showError(err.message);
        }
    };
}

// --- Password reset ---

function renderForgot(c) {
    const codeSent = mode === 'forgot-confirm';
    c.innerHTML = shell(`
        <h2 class="term-caps text-sm font-bold text-dim-500 mb-1">// Reset password</h2>
        <p class="text-gray-400 text-sm mb-6">${codeSent
            ? `Enter the code sent to <span class="text-white">${escHtml(pendingEmail)}</span> and pick a new password.`
            : "We'll email you a reset code."}</p>
        <form id="forgot-form">
            <div class="space-y-4">
                ${codeSent ? `
                <div>
                    <label class="term-caps block text-xs text-gray-400 mb-1.5">Reset code</label>
                    <input id="code" inputmode="numeric" autocomplete="one-time-code" required
                        class="${inputCls} tracking-[0.5em] text-center" placeholder="······">
                </div>
                <div>
                    <label class="term-caps block text-xs text-gray-400 mb-1.5">New password</label>
                    <input id="new-password" type="password" required minlength="8"
                        autocomplete="new-password" class="${inputCls}" placeholder="••••••••">
                </div>` : `
                <div>
                    <label class="term-caps block text-xs text-gray-400 mb-1.5">Email</label>
                    <input id="email" type="email" required value="${escHtml(pendingEmail)}"
                        class="${inputCls}" placeholder="you@company.com">
                </div>`}
                <div id="auth-error" class="text-red-400 text-sm hidden"></div>
                <button type="submit" class="${buttonCls}">
                    <span id="auth-btn-text">${codeSent ? '[ SET NEW PASSWORD ]' : '[ SEND RESET CODE ]'}</span>
                </button>
                <button type="button" id="back" class="term-caps w-full text-xs text-gray-500 hover:text-dim-400">Back to login</button>
            </div>
        </form>`);

    document.getElementById('back').onclick = () => { mode = 'login'; renderAuth(c); };
    document.getElementById('forgot-form').onsubmit = async (e) => {
        e.preventDefault();
        const btn = document.getElementById('auth-btn-text');
        try {
            if (!codeSent) {
                pendingEmail = document.getElementById('email').value.trim();
                btn.textContent = '[ SENDING… ]';
                await forgotPassword(pendingEmail);
                mode = 'forgot-confirm';
                renderAuth(c);
            } else {
                btn.textContent = '[ SAVING… ]';
                const code = document.getElementById('code').value.trim();
                const password = document.getElementById('new-password').value;
                await confirmForgotPassword(pendingEmail, code, password);
                await doLogin(pendingEmail, password);
            }
        } catch (err) {
            btn.textContent = codeSent ? '[ SET NEW PASSWORD ]' : '[ SEND RESET CODE ]';
            showError(err.message);
        }
    };
}
