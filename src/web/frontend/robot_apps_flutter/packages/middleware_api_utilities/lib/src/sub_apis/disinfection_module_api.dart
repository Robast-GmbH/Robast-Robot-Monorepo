import 'package:middleware_api_utilities/src/services/request_service.dart';

class DisinfectionModuleApi {
  DisinfectionModuleApi({required this.prefix});
  final String prefix;

  Future<bool> waitForDisinfectionTriggered({required String robotName, int timeout = 10}) async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/disinfection_triggered?robot_name=$robotName&timeout=$timeout'));
    return RequestService.wasRequestSuccessful(response: response);
  }
}
