import 'package:flutter/material.dart';
import 'package:robot_frontend/models/custom_focus_node.dart';
import 'package:virtual_keyboard_custom_layout/virtual_keyboard_custom_layout.dart';

class ModuleContentController {
  final initialItemsByCount = <String, int>{};
  final initialItemsByAmountFocusNode = <String, CustomFocusNode>{};
  final createdItemNameNodes = <CustomFocusNode>[];
  final createdItemAmountNodes = <CustomFocusNode>[];

  void setInitialItems(Map<String, int> itemsByCount) {
    for (final key in itemsByCount.keys) {
      initialItemsByCount[key] = itemsByCount[key]!;
      initialItemsByAmountFocusNode[key] = CustomFocusNode(
        key: GlobalKey(),
        text: itemsByCount[key].toString(),
        layout: VirtualKeyboardDefaultLayouts.Numeric,
      );
    }
  }

  void removeCreatedItem(int index) {
    if (index < 0 || index >= createdItemNameNodes.length) {
      return;
    }
    createdItemNameNodes.removeAt(index);
    createdItemAmountNodes.removeAt(index);
  }

  void createItem() {
    final amountNode = CustomFocusNode(
      key: GlobalKey(),
      text: '',
      layout: VirtualKeyboardDefaultLayouts.Numeric,
    );
    final textNode = CustomFocusNode(
      key: GlobalKey(),
      text: '',
      layout: VirtualKeyboardDefaultLayouts.German,
      next: amountNode,
    );
    createdItemNameNodes.add(textNode);
    createdItemAmountNodes.add(amountNode);
  }

  Map<String, int> createItemsByChange() {
    final tempCreatedItemsByCount = getCreatedItemsByCountWithCurrentInputs();

    final initialItemsByChange = <String, int>{};
    for (final entry in initialItemsByCount.entries) {
      final change = parseItemCount(initialItemsByAmountFocusNode[entry.key]?.text) - entry.value;
      initialItemsByChange[entry.key] = change;
    }

    final itemsByChange = <String, int>{};
    itemsByChange.addAll(tempCreatedItemsByCount);
    itemsByChange.addAll(initialItemsByChange);
    return itemsByChange;
  }

  bool didItemsChange() {
    final itemsByChange = createItemsByChange();
    return itemsByChange.isNotEmpty;
  }

  bool validateInputs({required String itemName, required String amount}) {
    if (itemName.trimLeft().trimRight().isEmpty || amount.isEmpty) {
      return false;
    }
    final amountNumber = int.tryParse(amount);
    if (amountNumber == null || amountNumber <= 0) {
      return false;
    }
    return true;
  }

  Map<String, int> getCreatedItemsByCountWithCurrentInputs() {
    final tempCreatedItemsByCount = <String, int>{};
    for (int i = 0; i < createdItemNameNodes.length; i++) {
      final itemName = createdItemNameNodes[i].text.trimLeft().trimRight();
      final itemAmount = parseItemCount(createdItemAmountNodes[i].text);
      if (itemName.isNotEmpty && itemAmount > 0 && !initialItemsByAmountFocusNode.containsKey(itemName)) {
        tempCreatedItemsByCount[itemName] = itemAmount;
      }
    }
    return tempCreatedItemsByCount;
  }

  int parseItemCount(String? text) {
    if (text == null) {
      return 0;
    }
    return int.tryParse(text) ?? 0;
  }
}
