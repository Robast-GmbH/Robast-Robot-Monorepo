class ModuleContentController {
  final initialItemsByCount = <String, int>{};
  final createdItemsByCount = <String, int>{};
  final contentItemsByChange = <String, int>{};

  void clear() {
    createdItemsByCount.clear();
    contentItemsByChange.clear();
  }

  Map<String, int> createItemsByChange() {
    final itemsByChange = <String, int>{};
    for (final entry in createdItemsByCount.entries) {
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
    final didCreateNonZeroItems = createdItemsByCount.entries.any((entry) => entry.value != 0);
    final didChangeContainingItemsCounts = contentItemsByChange.entries.any((entry) => entry.value != 0);
    return (createdItemsByCount.isNotEmpty && didCreateNonZeroItems) || (contentItemsByChange.isNotEmpty && didChangeContainingItemsCounts);
  }
}
