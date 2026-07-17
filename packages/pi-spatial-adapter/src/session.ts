import { AuthStorage, ModelRegistry, SessionManager, createAgentSession } from "@earendil-works/pi-coding-agent";
import type { Model } from "@earendil-works/pi-ai";
import { customTools, assertNoBuiltinTools, assertToolInventory } from "./tools.js";
import type { ToolDefinition } from "@earendil-works/pi-coding-agent";

export const MODEL_PROVIDER = "openai-codex";
export const MODEL_ID = "gpt-5.6-luna";
export const THINKING_LEVEL = "medium" as const;
export const REQUIRED_API = "openai-codex-responses";

export function resolveConfiguredModel(registry: ModelRegistry): Model<"openai-codex-responses"> {
  const model = registry.find(MODEL_PROVIDER, MODEL_ID);
  if (!model || model.api !== REQUIRED_API || !model.input.includes("image") || !model.reasoning) {
    throw new Error("configured Codex model is missing, has the wrong API, does not accept images, or does not support thinking");
  }
  return model as Model<"openai-codex-responses">;
}

export interface StoredAuthOptions {
  authPath: string;
  modelsPath?: string;
}

export interface SessionConfig {
  thinkingLevel: typeof THINKING_LEVEL;
}

export function validateSessionConfig(config: SessionConfig): void {
  if (config.thinkingLevel !== THINKING_LEVEL) throw new Error("unsupported thinking level");
}

export async function createFreshSession(tools: readonly ToolDefinition[], options: StoredAuthOptions, config: SessionConfig) {
  validateSessionConfig(config);
  const auth = AuthStorage.create(options.authPath);
  const credential = auth.get(MODEL_PROVIDER);
  if (!credential || credential.type !== "oauth") throw new Error("Codex OAuth credentials are not stored");
  const registry = ModelRegistry.create(auth, options.modelsPath);
  const model = resolveConfiguredModel(registry);
  if (!registry.isUsingOAuth(model)) throw new Error("configured model is not using Codex OAuth");
  const custom = customTools(tools);
  const available = custom.map((tool) => tool.name);
  assertNoBuiltinTools(available);
  if (available.length !== 3) throw new Error("exactly three custom tools are required");
  const result = await createAgentSession({
    model,
    thinkingLevel: config.thinkingLevel,
    authStorage: auth,
    modelRegistry: registry,
    sessionManager: SessionManager.inMemory(),
    noTools: "builtin",
    tools: available,
    customTools: custom,
  });
  const active = result.session.getActiveToolNames();
  assertToolInventory(active);
  return result.session;
}
