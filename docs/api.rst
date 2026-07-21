.. _dimos-api-reference:

=============
API Reference
=============

This reference is generated automatically from the docstrings in the source.

Blueprints and coordination
============================

.. autofunction:: dimos.core.coordination.blueprints.autoconnect

.. autoclass:: dimos.core.coordination.blueprints.Blueprint
   :members: transports
   :undoc-members:

.. autoclass:: dimos.core.coordination.module_coordinator.ModuleCoordinator
   :members: build, loop
   :undoc-members:

.. autoclass:: dimos.core.global_config.GlobalConfig

Modules and streams
===================

.. autofunction:: dimos.core.core.rpc

.. autoclass:: dimos.core.module.Module

.. autoclass:: dimos.core.stream.In
   :members: subscribe
   :undoc-members:

.. autoclass:: dimos.core.stream.Out
   :members: publish, subscribe
   :undoc-members:

Transports
==========

.. autoclass:: dimos.core.stream.Transport

.. autoclass:: dimos.core.transport.LCMTransport

.. autoclass:: dimos.core.coordination.blueprints.TransportSpec

Messages
========

.. autoclass:: dimos.msgs.geometry_msgs.Twist.Twist

.. autoclass:: dimos.msgs.sensor_msgs.Image.Image
   :members: from_numpy
   :undoc-members:

.. autoclass:: dimos.msgs.sensor_msgs.Image.ImageFormat

Agents and MCP
==============

.. autoclass:: dimos.agents.mcp.mcp_client.McpClient

.. autoclass:: dimos.agents.mcp.mcp_server.McpServer
