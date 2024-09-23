import 'package:flutter/material.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

class ModuleContentCreationView extends StatefulWidget {
  const ModuleContentCreationView({
    required this.moduleContentController,
    this.label = 'Fracht',
    super.key,
  });

  final String label;
  final ModuleContentController moduleContentController;

  @override
  State<ModuleContentCreationView> createState() => _ModuleContentCreationViewState();
}

class _ModuleContentCreationViewState extends State<ModuleContentCreationView> {
  final textController = TextEditingController();
  final amountController = TextEditingController();

  @override
  Widget build(BuildContext context) {
    final controller = widget.moduleContentController;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Padding(
          padding: const EdgeInsets.only(
            left: 8,
          ),
          child: Text(
            widget.label,
            style: const TextStyle(fontSize: 24, color: WebColors.primaryText),
          ),
        ),
        RoundedContainer(
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 8),
            child: Column(
              children: controller.createdItemsByCount.entries.map(buildContentListTile).toList() +
                  [
                    buildItemsByChangeCreationView(),
                    buildItemsByChangeCreationButton(),
                  ],
            ),
          ),
        ),
      ],
    );
  }

  Padding buildItemsByChangeCreationButton() {
    return Padding(
      padding: const EdgeInsets.only(top: 8),
      child: InkWell(
        onTap: () {
          setState(() {
            widget.moduleContentController.createdItemsByCount[textController.text] = int.tryParse(amountController.text) ?? 0;
            textController.clear();
            amountController.clear();
          });
        },
        child: const RoundedContainer(
          child: Padding(
            padding: EdgeInsets.symmetric(vertical: 8),
            child: SizedBox(
              width: double.infinity,
              child: Icon(
                Icons.add,
                color: WebColors.primaryIcon,
                size: 40,
              ),
            ),
          ),
        ),
      ),
    );
  }

  Widget buildContentListTile(MapEntry<String, int> entry) {
    final controller = widget.moduleContentController;
    return Padding(
      padding: const EdgeInsets.only(bottom: 8),
      child: RoundedContainer(
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
                    color: WebColors.secondaryIcon,
                  ),
                  buildListTileText(
                    (entry.value + (controller.contentItemsByChange.containsKey(entry.key) ? controller.contentItemsByChange[entry.key]! : 0)).toString(),
                  ),
                  IconButton(
                    iconSize: 24,
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
                    color: WebColors.secondaryIcon,
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  RoundedContainer buildItemsByChangeCreationView() {
    return RoundedContainer(
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 16),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Expanded(
              flex: 8,
              child: TextField(
                controller: textController,
                style: const TextStyle(fontSize: 18, color: WebColors.secondaryText),
              ),
            ),
            const SizedBox(
              width: 32,
            ),
            Expanded(
              child: TextField(
                textAlign: TextAlign.center,
                controller: amountController,
                style: const TextStyle(fontSize: 18, color: WebColors.secondaryText),
                keyboardType: const TextInputType.numberWithOptions(),
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
      style: const TextStyle(fontSize: 18, color: WebColors.secondaryText),
    );
  }
}
