import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/custom_focus_node.dart';
import 'package:robot_frontend/models/module_content_controller.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';
import 'package:robot_frontend/widgets/custom_textfield.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class ModuleContentCreationView extends StatefulWidget {
  const ModuleContentCreationView({
    required this.moduleContentController,
    this.label = 'Fracht',
    this.autoCreateFirstItem = false,
    this.showCategoryLabels = false,
    super.key,
  });

  final String label;
  final ModuleContentController moduleContentController;
  final bool autoCreateFirstItem;
  final bool showCategoryLabels;

  @override
  State<ModuleContentCreationView> createState() => _ModuleContentCreationViewState();
}

class _ModuleContentCreationViewState extends State<ModuleContentCreationView> {
  final scrollController = ScrollController();

  void createItem() {
    widget.moduleContentController.createItem();
    setState(() {});
    Provider.of<KeyboardProvider>(context, listen: false).focusNode = widget.moduleContentController.createdItemNameNodes.last;
    scrollController.jumpTo(scrollController.position.maxScrollExtent);
  }

  @override
  void initState() {
    if (widget.autoCreateFirstItem) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        createItem();
      });
    }
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    final contentController = widget.moduleContentController;
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
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              child: Column(
                children: [
                  Expanded(
                    child: ListView(controller: scrollController, children: [
                      if (widget.showCategoryLabels) ...[
                        buildListTileText('Vorhanden'),
                      ],
                      const SizedBox(
                        height: 4,
                      ),
                      ...contentController.initialItemsByChange.entries.map(
                        (entry) => buildContentListTile(entry.key, entry.value),
                      ),
                      if (widget.showCategoryLabels) ...[
                        const SizedBox(
                          height: 8,
                        ),
                        buildListTileText('Hinzufügen'),
                      ],
                      const SizedBox(
                        height: 4,
                      ),
                      ...List.generate(contentController.createdItemNameNodes.length, (index) {
                        return buildItemsByChangeCreationView(
                          textFocusNode: contentController.createdItemNameNodes[index],
                          amountFocusNode: contentController.createdItemAmountNodes[index],
                        );
                      }),
                      buildItemsByChangeCreationButton(),
                    ]),
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
      padding: const EdgeInsets.only(bottom: 96),
      child: InkWell(
        onTap: createItem,
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

  Widget buildContentListTile(String name, int amount) {
    final contentController = widget.moduleContentController;
    return Padding(
      padding: const EdgeInsets.only(bottom: 8),
      child: RoundedContainer(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Expanded(
                flex: 8,
                child: buildListTileText(name),
              ),
              const SizedBox(
                width: 32,
              ),
              buildListTileText((contentController.initialItemsByCount[name]! + amount).toString()),
              const SizedBox(
                width: 16,
              ),
              Container(
                decoration: BoxDecoration(
                  color: RobotColors.secondaryBackground,
                  borderRadius: BorderRadius.circular(20),
                ),
                child: Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 12),
                  child: Row(
                    children: [
                      Container(
                        decoration: BoxDecoration(
                          color: RobotColors.secondaryBackground,
                          borderRadius: BorderRadius.circular(20),
                        ),
                        child: Padding(
                          padding: const EdgeInsets.symmetric(horizontal: 4),
                          child: Icon(
                            amount < 0 ? Icons.arrow_downward : Icons.arrow_upward,
                            color: RobotColors.primaryIcon,
                            size: 30,
                          ),
                        ),
                      ),
                      const SizedBox(
                        width: 8,
                      ),
                      SizedBox(width: 44, child: buildListTileText(amount.abs().toString())),
                    ],
                  ),
                ),
              ),
              const SizedBox(
                width: 16,
              ),
              IconButton(
                  iconSize: 40,
                  onPressed: () {
                    Provider.of<KeyboardProvider>(context, listen: false).unfocus();
                    if (contentController.initialItemsByChange[name]! >= 99) {
                      return;
                    }
                    contentController.initialItemsByChange[name] = contentController.initialItemsByChange[name]! + 1;
                    setState(() {});
                  },
                  icon: const Icon(
                    Icons.add,
                    color: RobotColors.secondaryIcon,
                  )),
              IconButton(
                  iconSize: 40,
                  onPressed: () {
                    Provider.of<KeyboardProvider>(context, listen: false).unfocus();
                    if (contentController.initialItemsByChange[name]! + contentController.initialItemsByCount[name]! <= 0) {
                      return;
                    }
                    contentController.initialItemsByChange[name] = contentController.initialItemsByChange[name]! - 1;
                    setState(() {});
                  },
                  icon: const Icon(
                    Icons.remove,
                    color: RobotColors.secondaryIcon,
                  )),
              IconButton(
                  iconSize: 40,
                  onPressed: () {
                    Provider.of<KeyboardProvider>(context, listen: false).unfocus();
                    contentController.initialItemsByChange[name] = 0;
                    setState(() {});
                  },
                  icon: const Icon(
                    Icons.refresh,
                    color: RobotColors.secondaryIcon,
                  )),
            ],
          ),
        ),
      ),
    );
  }

  Widget buildItemsByChangeCreationView({required CustomFocusNode textFocusNode, required CustomFocusNode amountFocusNode}) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 8),
      child: RoundedContainer(
        child: Padding(
          padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 16),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Expanded(
                flex: 8,
                child: CustomTextfield(
                  focusNode: textFocusNode,
                ),
              ),
              const SizedBox(
                width: 32,
              ),
              Expanded(
                child: CustomTextfield(
                  focusNode: amountFocusNode,
                  mainAxisAlignment: MainAxisAlignment.center,
                ),
              ),
              const SizedBox(
                width: 16,
              ),
              IconButton(
                  iconSize: 40,
                  onPressed: () {
                    Provider.of<KeyboardProvider>(context, listen: false).unfocus();
                    final contenController = widget.moduleContentController;
                    contenController.createdItemNameNodes.remove(textFocusNode);
                    contenController.createdItemAmountNodes.remove(amountFocusNode);
                    setState(() {});
                  },
                  icon: const Icon(
                    Icons.delete,
                    color: RobotColors.secondaryIcon,
                  ))
            ],
          ),
        ),
      ),
    );
  }

  Text buildListTileText(String text) {
    return Text(
      text,
      style: const TextStyle(fontSize: 32, color: RobotColors.primaryText),
    );
  }
}
