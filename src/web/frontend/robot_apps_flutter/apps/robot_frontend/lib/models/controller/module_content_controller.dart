class ModuleContentController {
  final initialContent = <String, int>{};
  final createdContent = <String, int>{};
  final contentDifferences = <String, int>{};

  void clear() {
    createdContent.clear();
    contentDifferences.clear();
  }

  Map<String, int> createPayload() {
    final payload = <String, int>{};
    for (final entry in createdContent.entries) {
      payload[entry.key] = entry.value;
    }
    for (final entry in contentDifferences.entries) {
      if (payload.containsKey(entry.key)) {
        payload[entry.key] = payload[entry.key]! + entry.value;
      } else {
        payload[entry.key] = entry.value;
      }
    }
    return payload;
  }
}
