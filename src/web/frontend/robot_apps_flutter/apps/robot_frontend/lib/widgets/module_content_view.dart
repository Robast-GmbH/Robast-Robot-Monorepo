import 'package:flutter/material.dart';
import 'package:robot_frontend/models/controller/module_content_controller.dart';

class ModuleContentView extends StatefulWidget {
  const ModuleContentView({
    required this.moduleContentController,
    this.label = 'Fracht',
    super.key,
  });

  final String label;
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
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Padding(
          padding: EdgeInsets.only(
            left: 8,
          ),
          child: Text('Fracht', style: TextStyle(fontSize: 28)),
        ),
        Expanded(
          child: Card(
            color: Colors.white.withOpacity(0.4),
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
              child: Column(
                children: [
                  Expanded(
                    child: ListView(
                      children: widget.moduleContentController.initialItemsByCount.entries.map<Widget>(buildContentListTile).toList() +
                          controller.createdItemsByCount.entries.map(buildContentListTile).toList() +
                          [
                            buildItemsByChangeCreationView(),
                            buildItemsByChangeCreationButton(),
                          ],
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }

  Padding buildItemsByChangeCreationButton() {
    return Padding(
      padding: const EdgeInsets.only(bottom: 32),
      child: Card(
        color: Colors.white.withOpacity(0.7),
        child: IconButton(
          color: Colors.white,
          iconSize: 64,
          onPressed: () {
            setState(() {
              widget.moduleContentController.createdItemsByCount[textController.text] = int.tryParse(amountController.text) ?? 0;
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
                    if (controller.contentItemsByChange.containsKey(entry.key)) {
                      setState(() {
                        controller.contentItemsByChange[entry.key] = controller.contentItemsByChange[entry.key]! - 1;
                      });
                    } else {
                      setState(() {
                        controller.contentItemsByChange[entry.key] = -1;
                      });
                    }
                  },
                  icon: const Icon(Icons.remove),
                ),
                buildListTileText(
                  (entry.value + (controller.contentItemsByChange.containsKey(entry.key) ? controller.contentItemsByChange[entry.key]! : 0)).toString(),
                ),
                IconButton(
                  iconSize: 32,
                  onPressed: () {
                    if (controller.contentItemsByChange.containsKey(entry.key)) {
                      setState(() {
                        controller.contentItemsByChange[entry.key] = controller.contentItemsByChange[entry.key]! + 1;
                      });
                    } else {
                      setState(() {
                        controller.contentItemsByChange[entry.key] = 1;
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

  Card buildItemsByChangeCreationView() {
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
