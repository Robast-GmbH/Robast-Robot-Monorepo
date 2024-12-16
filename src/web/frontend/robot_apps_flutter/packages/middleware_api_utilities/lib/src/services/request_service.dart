import 'dart:async';
import 'dart:convert';
import 'package:http/http.dart' as http;

class RequestService {
  static Future<http.Response?> tryGet({
    required Uri uri,
    int timeoutInMS = 200,
  }) async {
    final client = http.Client();
    try {
      final response = await client.get(
        uri,
        headers: {
          'charset': 'utf-8',
        },
      ).timeout(Duration(milliseconds: timeoutInMS));

      if (response.statusCode != 200) {
        return null;
      }
      return response;
    } catch (e) {
      return null;
    } finally {
      client.close();
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
      final response = await http
          .post(
            uri,
            headers: headers,
            body: data != null ? jsonEncode(data) : null,
          )
          .timeout(const Duration(seconds: 10));
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
      final response = await http
          .put(
            uri,
            headers: headers,
            body: data != null ? jsonEncode(data) : null,
          )
          .timeout(const Duration(seconds: 10));
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
