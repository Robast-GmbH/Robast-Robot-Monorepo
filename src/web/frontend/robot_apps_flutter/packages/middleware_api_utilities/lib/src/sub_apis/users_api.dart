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
}
