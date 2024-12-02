import 'dart:convert';

import 'package:middleware_api_utilities/src/services/request_service.dart';

class LogsApi {
  LogsApi({required this.prefix});
  final String prefix;

  Future<List<String>?> getAvailableLogs() async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/logs/'));
    if (response != null && response.statusCode == 200) {
      return ((jsonDecode(response.body) as Map<String, dynamic>)['logs'] as List<dynamic>).cast<String>();
    } else {
      return null;
    }
  }

  Future<String?> getLog(String path) async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/logs/$path'));
    if (response != null && response.statusCode == 200) {
      return response.body;
    } else {
      return null;
    }
  }
}
