import 'dart:convert';

import 'package:middleware_api_utilities/src/models/robot_drawer.dart';
import 'package:http/http.dart' as http;

class ModulesApi {
  final String prefix;

  ModulesApi({required this.prefix});

  Future<http.Response?> tryGet({
    required Uri uri,
  }) async {
    try {
      final response = await http.get(uri);
      if (response.statusCode == 200) {
        return response;
      } else {
        return null;
      }
    } catch (e) {
      return null;
    }
  }

  Future<http.Response?> tryPost({
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

  bool _wasRequestSuccessful({
    required http.Response? response,
  }) {
    if (response != null) {
      final data = jsonDecode(response.body);
      return data['status'] == 'success';
    } else {
      return false;
    }
  }

  Future<List<RobotDrawer>> getModules({
    required String robotName,
  }) async {
    final drawers = <RobotDrawer>[];
    final response = await tryGet(uri: Uri.parse('$prefix/modules?robot_name=$robotName'));
    if (response != null) {
      final jsonData = jsonDecode(response.body) as List<dynamic>;
      for (final drawer in jsonData) {
        drawers.add(RobotDrawer.fromJson(drawer as Map<String, dynamic>));
      }
    }
    return drawers;
  }

  Future<bool> createModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
    required int position,
    required int size,
    required String variant,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/create_module'),
      data: {
        'drawer_address': {
          'robot_name': robotName,
          'module_id': moduleID,
          'drawer_id': drawerID,
        },
        'position': position,
        'size': size,
        'variant': variant,
      },
    );
    return _wasRequestSuccessful(response: response);
  }

  Future<bool> deleteModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/delete_module'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return _wasRequestSuccessful(response: response);
  }

  Future<bool> emptyModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/empty_module'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return _wasRequestSuccessful(response: response);
  }

  Future<bool> updateModuleContent({
    required String robotName,
    required int moduleID,
    required int drawerID,
    required String itemID,
    required int quantity,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/update_module_content'),
      data: {
        'drawer_address': {
          'robot_name': robotName,
          'module_id': moduleID,
          'drawer_id': drawerID,
        },
        'item_id': itemID,
        'quantity': quantity,
      },
    );
    return _wasRequestSuccessful(response: response);
  }

  Future<bool> freeModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/free_module'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return _wasRequestSuccessful(response: response);
  }

  Future<bool> reserveModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
    required List<String> userIDs,
    required List<String> userGroups,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/reserve_module'),
      data: {
        'drawer_address': {
          'robot_name': robotName,
          'module_id': moduleID,
          'drawer_id': drawerID,
        },
        'user_ids': userIDs,
        'user_groups': userGroups,
      },
    );
    return _wasRequestSuccessful(response: response);
  }

  Future<bool> startModuleProcess({
    required String robotName,
    required int moduleID,
    required int drawerID,
    required String processName,
    required Map<String, int> payload,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/start_module_process'),
      data: {
        'robot_name': robotName,
        'module_process_data': {
          'module_id': moduleID,
          'drawer_id': drawerID,
          'process_name': processName,
          'payload': payload,
        }
      },
    );
    return _wasRequestSuccessful(response: response);
  }

  Future<bool> openDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/open_drawer'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return _wasRequestSuccessful(response: response);
  }

  Future<bool> closeDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/close_drawer'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return _wasRequestSuccessful(response: response);
  }

  Future<bool> finishModuleProcess({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await tryPost(
      uri: Uri.parse('$prefix/modules/finish_module_process'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return _wasRequestSuccessful(response: response);
  }
}
