import test from "node:test";
import assert from "node:assert/strict";
import { TOOL_NAMES, assertNoBuiltinTools, assertToolInventory, submitAnswerParameters, toolDefinitions, createBroker } from "../src/tools.js";

test("accepts exactly the three host-supplied tools", () => {
  assert.doesNotThrow(() => assertToolInventory(TOOL_NAMES));
  assert.throws(() => assertToolInventory(["sandbox_exec", "render_case"]));
  assert.throws(() => assertToolInventory(["sandbox_exec", "render_case", "submit_answer", "bash"]));
  assert.doesNotThrow(() => assertNoBuiltinTools(TOOL_NAMES));
  assert.throws(() => assertNoBuiltinTools(["read"]));
});

test("uses workspace image paths and public answer-type schemas", () => {
  const broker = createBroker(() => undefined, 1);
  const booleanSubmit = toolDefinitions(broker, "boolean").find((tool) => tool.name === "submit_answer");
  const integerSubmit = toolDefinitions(broker, "integer").find((tool) => tool.name === "submit_answer");
  assert.deepEqual(booleanSubmit?.parameters, submitAnswerParameters("boolean"));
  assert.deepEqual(integerSubmit?.parameters, submitAnswerParameters("integer"));
  const image = toolDefinitions(broker, "boolean").find((tool) => tool.name === "read_generated_image");
  assert.match(JSON.stringify(image?.parameters), /"path"/);
  assert.match(JSON.stringify(image?.parameters), /512/);
  assert.equal(JSON.stringify(image?.parameters).includes("\\\\.\\\\."), true);
});
