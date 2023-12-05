import 'dart:io';

import 'package:flutter/material.dart';

class ScriptButton extends StatefulWidget {
  const ScriptButton({
    super.key,
    required this.arguments,
    required this.title,
  });
  final String title;
  final List<String> arguments;

  @override
  State<ScriptButton> createState() => _ScriptButtonState();
}

class _ScriptButtonState extends State<ScriptButton> {
  void showResultDialog(ProcessResult result) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text("Script ausgeführt"),
        content: Text(
          result.exitCode == 0 ? result.stdout : result.stderr,
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: ListTile(
        title: Text(widget.title),
        onTap: () async {
          final result = await Process.run('bash', widget.arguments);
          showResultDialog(result);
        },
      ),
    );
  }
}
