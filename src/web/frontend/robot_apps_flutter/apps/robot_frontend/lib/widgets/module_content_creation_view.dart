import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/custom_textfield.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:shared_data_models/shared_data_models.dart';

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
  final textController = TextController();
  final amountController = TextController();
  final scrollController = ScrollController();
  bool createdAnItem = false;

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
            style: const TextStyle(fontSize: 28, color: RobotColors.primaryText),
          ),
        ),
        Expanded(
          child: RoundedContainer(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
              child: Column(
                children: [
                  Expanded(
                    child: ListView(
                      controller: scrollController,
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
      padding: const EdgeInsets.only(bottom: 96, top: 8),
      child: InkWell(
        onTap: () {
          final amount = int.tryParse(amountController.text);
          if (textController.text.trimRight().trimLeft().isEmpty || amountController.text.isEmpty || amount == null) {
            return;
          }
          setState(() {
            createdAnItem = true;
            widget.moduleContentController.createdItemsByCount[textController.text] = amount;
            textController.clear();
            amountController.clear();
          });
          scrollController.jumpTo(scrollController.position.maxScrollExtent);
        },
        child: const RoundedContainer(
          child: Padding(
            padding: EdgeInsets.symmetric(vertical: 4),
            child: Icon(
              Icons.add,
              color: RobotColors.primaryIcon,
              size: 64,
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
              padding: const EdgeInsets.all(12),
              child: buildListTileText(entry.key),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16),
              child: Row(
                children: [
                  IconButton(
                    iconSize: 48,
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
                    color: RobotColors.secondaryIcon,
                  ),
                  Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 8),
                    child: buildListTileText(
                      (entry.value + (controller.contentItemsByChange.containsKey(entry.key) ? controller.contentItemsByChange[entry.key]! : 0)).toString(),
                    ),
                  ),
                  IconButton(
                    iconSize: 48,
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
                    color: RobotColors.secondaryIcon,
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
              child: CustomTextfield(
                textController: textController,
                enabledAutofocus: createdAnItem,
              ),
            ),
            const SizedBox(
              width: 32,
            ),
            Expanded(
              child: CustomTextfield(
                textController: amountController,
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
      style: const TextStyle(fontSize: 32, color: RobotColors.secondaryText),
    );
  }
}
