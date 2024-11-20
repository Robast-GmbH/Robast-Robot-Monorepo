import 'dart:convert';
import 'dart:typed_data';

import 'package:middleware_api_utilities/src/services/request_service.dart';

class ManualsApi {
  ManualsApi({required this.prefix});
  final String prefix;

  Future<List<Map<String, dynamic>>?> fetchManualsList() async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/manuals/'));
    if (response != null && response.statusCode == 200) {
      return ((jsonDecode(response.body) as Map<String, dynamic>)['files'] as List<dynamic>).cast<Map<String, dynamic>>();
    } else {
      return null;
    }
  }

  Future<Uint8List?> downloadManual(String path) async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/manuals/$path'), timeout: 5);
    if (response != null && response.statusCode == 200) {
      return response.bodyBytes;
    } else {
      return null;
    }
  }
}
