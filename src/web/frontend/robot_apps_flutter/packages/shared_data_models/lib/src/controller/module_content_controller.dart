class ModuleContentController {
  final initialItemsByCount = <String, int>{};
  final createdItemsByCount = <String, int>{};
  final contentItemsByChange = <String, int>{};

  String itemName = '';
  String amount = '';

  void clear() {
    createdItemsByCount.clear();
    contentItemsByChange.clear();
  }

  Map<String, int> createItemsByChange() {
    final tempCreatedItemsByCount = getCreatedItemsByCountWithCurrentInputs();
    final itemsByChange = <String, int>{};
    for (final entry in tempCreatedItemsByCount.entries) {
      itemsByChange[entry.key] = entry.value;
    }
    for (final entry in contentItemsByChange.entries) {
      if (itemsByChange.containsKey(entry.key)) {
        itemsByChange[entry.key] = itemsByChange[entry.key]! + entry.value;
      } else {
        itemsByChange[entry.key] = entry.value;
      }
    }
    return itemsByChange;
  }

  bool didItemsChange() {
    final tempCreatedItemsByCount = getCreatedItemsByCountWithCurrentInputs();
    final didCreateNonZeroItems = tempCreatedItemsByCount.entries.any((entry) => entry.value != 0);
    final didChangeContainingItemsCounts = contentItemsByChange.entries.any((entry) => entry.value != 0);
    return (tempCreatedItemsByCount.isNotEmpty && didCreateNonZeroItems) || (contentItemsByChange.isNotEmpty && didChangeContainingItemsCounts);
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
    for (final entry in createdItemsByCount.entries) {
      tempCreatedItemsByCount[entry.key] = entry.value;
    }
    if (validateInputs(itemName: itemName, amount: amount)) {
      if (tempCreatedItemsByCount.containsKey(itemName)) {
        tempCreatedItemsByCount[itemName] = tempCreatedItemsByCount[itemName]! + int.parse(amount);
      } else {
        tempCreatedItemsByCount[itemName] = int.parse(amount);
      }
    }
    return tempCreatedItemsByCount;
  }
}
