import 'dart:convert';

import 'package:middleware_api_utilities/src/services/request_service.dart';

class FireAlarmApi {
  FireAlarmApi({required this.prefix});
  final String prefix;

  Future<bool?> fireAlarmTriggered() async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/fire_alarm/fire_alarm_triggered'));
    if (response != null && response.statusCode == 200) {
      return (jsonDecode(response.body) as Map<String, dynamic>)['fire_alarm_triggered'] as bool;
    } else {
      return null;
    }
  }
}
