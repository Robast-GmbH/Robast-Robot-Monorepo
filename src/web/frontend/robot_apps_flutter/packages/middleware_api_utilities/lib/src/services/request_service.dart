import 'dart:convert';

import 'package:http/http.dart' as http;

class RequestService {
  static Future<http.Response?> tryGet({
    required Uri uri,
    int timeout = 1,
  }) async {
    try {
      final response = await http.get(
        uri,
        headers: {
          'charset': 'utf-8',
        },
      ).timeout(Duration(seconds: timeout));

      if (response.statusCode == 200) {
        return response;
      } else {
        return null;
      }
    } catch (e) {
      return null;
    }
  }

  static Future<http.Response?> tryPost({
    required Uri uri,
    Map<String, dynamic>? data,
  }) async {
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };
    try {
      final response = await http.post(
        uri,
        headers: headers,
        body: data != null ? jsonEncode(data) : null,
      );
      if (response.statusCode == 200) {
        return response;
      } else {
        return null;
      }
    } catch (e) {
      return null;
    }
  }

  static Future<http.Response?> tryPut({
    required Uri uri,
    Map<String, dynamic>? data,
  }) async {
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };
    try {
      final response = await http.put(
        uri,
        headers: headers,
        body: data != null ? jsonEncode(data) : null,
      );
      if (response.statusCode == 200) {
        return response;
      } else {
        return null;
      }
    } catch (e) {
      return null;
    }
  }

  static bool wasRequestSuccessful({
    required http.Response? response,
  }) {
    if (response != null) {
      final data = jsonDecode(response.body) as Map<String, dynamic>;
      return data['status'] == 'success';
    } else {
      return false;
    }
  }

  static Map<String, dynamic> responseToMap({required http.Response response}) {
    final jsonData = utf8.decode(response.bodyBytes);
    final dataMap = jsonDecode(jsonData) as Map<String, dynamic>;
    return dataMap;
  }

  static List<dynamic> responseToList({required http.Response response}) {
    final jsonData = utf8.decode(response.bodyBytes);
    final dataList = jsonDecode(jsonData) as List<dynamic>;
    return dataList;
  }
}
