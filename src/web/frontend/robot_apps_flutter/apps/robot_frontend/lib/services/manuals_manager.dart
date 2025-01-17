import 'dart:io';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:path_provider/path_provider.dart';

class ManualsManager {
  Future<String> getCacheDir() async {
    final directory = await getApplicationDocumentsDirectory();
    Directory('${directory.path}/manuals').createSync();
    return '${directory.path}/manuals';
  }

  Future<bool> isFileUpToDate(String fileName, double lastModifiedTimestamp) async {
    final cacheDir = await getCacheDir();
    final filePath = '$cacheDir/$fileName';
    final file = File(filePath);

    if (!file.existsSync()) return false;

    final cachedLastModified = file.lastModifiedSync();
    final apiLastModified = DateTime.fromMillisecondsSinceEpoch((lastModifiedTimestamp * 1000).toInt());

    return cachedLastModified.isAtSameMomentAs(apiLastModified) || cachedLastModified.isAfter(apiLastModified);
  }

  Future<void> downloadAndCacheFile(String fileName, double lastModifiedTimestamp) async {
    final cacheDir = await getCacheDir();
    final filePath = '$cacheDir/$fileName';

    final middlewareApi = MiddlewareApiUtilities();
    final manualData = await middlewareApi.manuals.downloadManual(fileName);
    if (manualData != null) {
      final file = File(filePath);
      await file.writeAsBytes(manualData);
      final apiLastModified = DateTime.now();
      file.setLastModifiedSync(apiLastModified);
    } else {
      throw Exception('Failed to download file: $fileName');
    }
  }

  Future<void> updateManuals() async {
    final middlewareApi = MiddlewareApiUtilities();
    final manualsList = await middlewareApi.manuals.fetchManualsList();
    if (manualsList == null) return;

    for (final file in manualsList) {
      final fileName = file['name']!;
      final lastModified = file['last_modified']!;

      final isUpToDate = await isFileUpToDate(fileName, lastModified);

      if (!isUpToDate) {
        await downloadAndCacheFile(fileName, lastModified);
      }
    }

    final cachedFiles = await getManualsList();
    for (final filePath in cachedFiles) {
      final isAvailable = manualsList.any((element) => element['name'] == filePath);
      if (!isAvailable) {
        final cacheDir = await getCacheDir();
        final completeFilePath = '$cacheDir/$filePath';
        final file = File(completeFilePath);
        file.deleteSync();
      }
    }
  }

  Future<List<String>> getManualsList() async {
    final cacheDir = await getCacheDir();
    final dir = Directory(cacheDir);
    final files = dir.listSync();

    final fileNames = <String>[];
    for (final file in files) {
      if (file is File && file.path.endsWith('.pdf')) {
        fileNames.add(file.path.split('/').last);
      }
    }

    return fileNames;
  }
}
