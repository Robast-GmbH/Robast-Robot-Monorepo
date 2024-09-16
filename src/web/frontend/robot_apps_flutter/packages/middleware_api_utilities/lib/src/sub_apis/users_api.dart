import 'dart:convert';

import 'package:middleware_api_utilities/src/models/user.dart';
import 'package:middleware_api_utilities/src/services/request_service.dart';

class UsersApi {
  UsersApi({required this.prefix});

  final String prefix;

  Future<List<User>> getUsers() async {
    final users = <User>[];
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/users/all_users'));
    if (response != null) {
      final data = RequestService.responseToList(response: response);
      for (final user in data) {
        users.add(User.fromJson(user as Map<String, dynamic>));
      }
    }
    return users;
  }

  Future<User?> getUser({
    required String id,
  }) async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/users/user?user_id=$id'));
    if (response != null) {
      final data = RequestService.responseToMap(response: response);
      return User.fromJson(data);
    } else {
      return null;
    }
  }

  Future<User?> updateUser({required User updatedUser}) async {
    final response = await RequestService.tryPut(uri: Uri.parse('$prefix/users/update_user'), data: updatedUser.toJson());
    if (response != null) {
      final jsonData = jsonDecode(response.body) as Map<String, dynamic>;
      return User.fromJson(jsonData);
    } else {
      return null;
    }
  }

  Future<User?> createUser({required User newUser}) async {
    final response = await RequestService.tryPost(uri: Uri.parse('$prefix/users/create_user'), data: newUser.toJson());
    if (response != null) {
      final jsonData = jsonDecode(response.body) as Map<String, dynamic>;
      return User.fromJson(jsonData);
    } else {
      return null;
    }
  }

  Future<bool> deleteUser({required String id}) async {
    final response = await RequestService.tryPost(uri: Uri.parse('$prefix/users/delete_user'), data: {'id': id});
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> readAndAssignUserNFC({
    required String robotName,
    required String userID,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/users/read_and_assign_user_nfc_id?robot_name=$robotName&user_id=$userID'),
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> setUserSession({required String robotName, required String userID}) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/users/set_session?robot_name=$robotName&user_id=$userID'),
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<User?> getUserSession({required String robotName}) async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/users/session?robot_name=$robotName'));
    if (response != null) {
      final data = RequestService.responseToMap(response: response);
      if (data['user'] != null) {
        return User.fromJson(data['user'] as Map<String, dynamic>);
      }
    }
    return null;
  }

  Future<bool> tryStartUserSession({
    required String robotName,
    List<String> requiredUserIDs = const [],
    List<String> requiredUserGroups = const [],
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/users/try_start_session?robot_name=$robotName'),
      data: {
        'required_user_ids': requiredUserIDs,
        'required_user_groups': requiredUserGroups,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> endUserSession({required String robotName}) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/users/end_session?robot_name=$robotName'),
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<User?> loginUser({
    required String mail,
    required String password,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/users/login'),
      data: {
        'mail': mail,
        'password': password,
      },
    );
    if (response != null) {
      final data = RequestService.responseToMap(response: response);
      if (data['user'] != null) {
        return User.fromJson(data['user'] as Map<String, dynamic>);
      }
      return null;
    }
    return null;
  }

  Future<bool> changePassword({required String userID, required String oldPassword, required String newPassword}) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/users/change_password?user_id=$userID&old_password=$oldPassword&new_password=$newPassword'),
      data: {},
    );
    return RequestService.wasRequestSuccessful(response: response);
  }
}
