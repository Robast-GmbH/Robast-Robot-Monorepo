import 'package:flutter/material.dart';
import 'package:robot_frontend/models/controller/module_content_controller.dart';

class ModuleContentView extends StatefulWidget {
  const ModuleContentView({required this.moduleContentController, super.key});

  final ModuleContentController moduleContentController;

  @override
  State<ModuleContentView> createState() => _ModuleContentViewState();
}

class _ModuleContentViewState extends State<ModuleContentView> {
  final textController = TextEditingController();
  final amountController = TextEditingController();

  @override
  Widget build(BuildContext context) {
    final controller = widget.moduleContentController;
    return ListView(
      children: widget.moduleContentController.initialContent.entries.map<Widget>(buildContentListTile).toList() +
          controller.createdContent.entries.map(buildContentListTile).toList() +
          [
            buildPayloadCreationView(),
            buildPayloadCreationButton(),
          ],
    );
  }

  Padding buildPayloadCreationButton() {
    return Padding(
      padding: const EdgeInsets.only(bottom: 32),
      child: Card(
        color: Colors.white.withOpacity(0.7),
        child: IconButton(
          color: Colors.white,
          iconSize: 64,
          onPressed: () {
            setState(() {
              widget.moduleContentController.createdContent[textController.text] = int.tryParse(amountController.text) ?? 0;
              textController.clear();
              amountController.clear();
            });
          },
          icon: const Icon(Icons.add),
        ),
      ),
    );
  }

  Card buildContentListTile(MapEntry<String, int> entry) {
    final controller = widget.moduleContentController;
    return Card(
      color: Colors.white.withOpacity(0.5),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Padding(
            padding: const EdgeInsets.all(16),
            child: buildListTileText(entry.key),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            child: Row(
              children: [
                IconButton(
                  iconSize: 32,
                  onPressed: () {
                    if (controller.contentDifferences.containsKey(entry.key)) {
                      setState(() {
                        controller.contentDifferences[entry.key] = controller.contentDifferences[entry.key]! - 1;
                      });
                    } else {
                      setState(() {
                        controller.contentDifferences[entry.key] = -1;
                      });
                    }
                  },
                  icon: const Icon(Icons.remove),
                ),
                buildListTileText(
                  (entry.value + (controller.contentDifferences.containsKey(entry.key) ? controller.contentDifferences[entry.key]! : 0)).toString(),
                ),
                IconButton(
                  iconSize: 32,
                  onPressed: () {
                    if (controller.contentDifferences.containsKey(entry.key)) {
                      setState(() {
                        controller.contentDifferences[entry.key] = controller.contentDifferences[entry.key]! + 1;
                      });
                    } else {
                      setState(() {
                        controller.contentDifferences[entry.key] = 1;
                      });
                    }
                  },
                  icon: const Icon(Icons.add),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Card buildPayloadCreationView() {
    return Card(
      color: Colors.white.withOpacity(0.5),
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Expanded(
              flex: 8,
              child: TextField(
                controller: textController,
                style: const TextStyle(fontSize: 24),
              ),
            ),
            const SizedBox(
              width: 32,
            ),
            Expanded(
              child: TextField(
                textAlign: TextAlign.center,
                controller: amountController,
                style: const TextStyle(fontSize: 24),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Text buildListTileText(String text) {
    return Text(
      text,
      style: const TextStyle(fontSize: 24),
    );
  }
}
