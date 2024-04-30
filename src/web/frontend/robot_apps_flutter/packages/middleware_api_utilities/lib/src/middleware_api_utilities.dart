import 'package:http/http.dart' as http;
class MiddlewareApiUtilities {
  /// {@macro middleware_api_utilities}
  const MiddlewareApiUtilities(this._serverAddress);

  final String _serverAddress;

  /// Check if the server is available
  Future<bool> isServerAvailable() async {
    final response = await http.get(Uri.parse('http://$_serverAddress/'),headers: {
      'Access-Control-Allow-Origin': '*',
    });
    if (response.statusCode != 200) {
      return false;
    }
    return true;
  }
}
