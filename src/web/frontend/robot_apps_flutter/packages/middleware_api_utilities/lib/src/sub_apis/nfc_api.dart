import 'package:middleware_api_utilities/src/services/request_service.dart';

class NFCApi {
  NFCApi({required this.prefix});
  final String prefix;

  Future<bool> writeNFC({
    required String robotName,
    required String nfcData,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/write_nfc_tag?nfc_tag_id=$nfcData&robot_name=$robotName'),
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<String> readNFC({required String robotName}) async {
    final response = await RequestService.tryGet(
      uri: Uri.parse('$prefix/read_nfc_tag?robot_name=$robotName'),
      timeout: 30,
    );
    if (response != null) {
      final data = RequestService.responseToMap(response: response);
      return (data['nfc_tag'] as Map<String, dynamic>)['data'] as String;
    } else {
      return '';
    }
  }
}
