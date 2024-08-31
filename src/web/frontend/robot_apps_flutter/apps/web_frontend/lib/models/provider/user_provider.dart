import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class UserProvider extends ChangeNotifier {
  UserProvider({required String prefix}) {
    _middlewareApi = MiddlewareApiUtilities();
    _middlewareApi.setPrefix(prefix: prefix);
  }

  late MiddlewareApiUtilities _middlewareApi;
  User? user;

  Future<List<User>> getUsers() async {
    return _middlewareApi.users.getUsers();
  }

  Future<bool> login({required String mail, required String password}) async {
    user = await MiddlewareApiUtilities().users.loginUser(mail: mail, password: password);
    return user != null;
  }

  Future<bool> createUser({required User newUser}) async {
    final result = await _middlewareApi.users.createUser(newUser: newUser);
    return result != null;
  }

  Future<bool> updateUser({required User updatedUser}) async {
    final result = await _middlewareApi.users.updateUser(updatedUser: updatedUser);
    if (result != null) {
      user = result;
      notifyListeners();
    }
    return result != null;
  }

  Future<bool> deleteUser({required String userID}) async {
    final result = await _middlewareApi.users.deleteUser(id: userID);
    return result;
  }

  Future<bool> changePassword({required String oldPassword, required String newPassword}) async {
    if (user == null) {
      return false;
    }
    final result = await _middlewareApi.users.changePassword(
      userID: user!.id,
      oldPassword: oldPassword,
      newPassword: newPassword,
    );
    return result;
  }
}
