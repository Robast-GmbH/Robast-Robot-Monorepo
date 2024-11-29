import 'dart:convert';

import 'package:middleware_api_utilities/src/services/request_service.dart';

class HygieneApi {
  HygieneApi({required this.prefix});

  final String prefix;

  /// Set the cleaning cycle for a robot in hours.
  Future<bool> setCycle({required String robotName, required double cycleTimeInH}) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/hygiene/set_cycle/$robotName?cycle=$cycleTimeInH'),
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  /// Get the cleaning cycle for a robot.
  Future<double?> getCycle({required String robotName}) async {
    final response = await RequestService.tryGet(
      uri: Uri.parse('$prefix/hygiene/get_cycle/$robotName'),
    );
    if (response != null && response.statusCode == 200) {
      return (jsonDecode(response.body) as Map<String, dynamic>)['cycle'] as double;
    } else {
      return null;
    }
  }

  /// Set the last cleaning timestamp for a robot to the current time.
  Future<bool> setLastCleaning({required String robotName}) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/hygiene/set_last_cleaning/$robotName'),
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  /// Get the last cleaning timestamp for a robot.
  Future<DateTime?> getLastCleaning({required String robotName}) async {
    final response = await RequestService.tryGet(
      uri: Uri.parse('$prefix/hygiene/get_last_cleaning/$robotName'),
    );
    if (response != null && response.statusCode == 200) {
      final lastCleaning = (jsonDecode(response.body) as Map<String, dynamic>)['last_cleaning'] as String;
      return DateTime.parse(lastCleaning);
    } else {
      return null;
    }
  }

  /// Check if a robot requires cleaning based on the cycle and last cleaning timestamp.
  Future<bool?> getRequiresCleaning({required String robotName}) async {
    final response = await RequestService.tryGet(
      uri: Uri.parse('$prefix/hygiene/requires_cleaning/$robotName'),
    );
    if (response != null && response.statusCode == 200) {
      return (jsonDecode(response.body) as Map<String, dynamic>)['requires_cleaning'] as bool;
    } else {
      return null;
    }
  }

  /// Get the next cleaning timestamp for a robot based on the cycle and last cleaning timestamp.
  Future<DateTime?> getNextCleaning({required String robotName}) async {
    final response = await RequestService.tryGet(
      uri: Uri.parse('$prefix/hygiene/requires_cleaning/$robotName'),
    );
    if (response != null && response.statusCode == 200) {
      final nextCleaning = (jsonDecode(response.body) as Map<String, dynamic>)['next_cleaning_due'] as String;
      final timeZoneOffset = DateTime.now().timeZoneOffset;
      return DateTime.parse(nextCleaning).add(Duration(hours: timeZoneOffset.inHours));
    } else {
      return null;
    }
  }

  Future<bool> setRequiresDisinfectionAfterUsage({required String robotName, required bool requiresDisinfection}) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/hygiene/set_requires_disinfection_after_usage/$robotName?requires_disinfection=$requiresDisinfection'),
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool?> getRequiresDisinfectionAfterUsage({required String robotName}) async {
    final response = await RequestService.tryGet(
      uri: Uri.parse('$prefix/hygiene/get_requires_disinfection_after_usage/$robotName'),
    );
    if (response != null && response.statusCode == 200) {
      return (jsonDecode(response.body) as Map<String, dynamic>)['requires_disinfection_after_usage'] as bool;
    } else {
      return null;
    }
  }
}
