import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

class AuthView extends StatefulWidget {
  const AuthView({
    required this.requestedUserIDs,
    required this.requestedUserGroups,
    super.key,
  });
  final List<String> requestedUserIDs;
  final List<String> requestedUserGroups;

  @override
  State<AuthView> createState() => _AuthViewState();
}

class _AuthViewState extends State<AuthView> {
  late Future<List<User>> loadUsers;

  @override
  void initState() {
    super.initState();
    if (widget.requestedUserIDs.isNotEmpty || widget.requestedUserGroups.isNotEmpty) {
      loadUsers = Provider.of<UserProvider>(context, listen: false).getUsers();
    }
  }

  @override
  Widget build(BuildContext context) {
    if (widget.requestedUserIDs.isEmpty && widget.requestedUserGroups.isEmpty) {
      return const Text(
        'Bitte authentifizieren Sie sich mit Ihrem NFC-Tag',
        style: TextStyle(
          fontSize: 50,
          fontWeight: FontWeight.bold,
        ),
      );
    }
    return Center(
      child: FutureBuilder<List<User>>(
        future: loadUsers,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const CircularProgressIndicator();
          }

          if (widget.requestedUserIDs.isEmpty && widget.requestedUserGroups.isNotEmpty) {
            return Text(
              'Bitte melden Sie sich an ${widget.requestedUserGroups.join(', ')}',
              style: const TextStyle(
                fontSize: 50,
                fontWeight: FontWeight.bold,
              ),
            );
          }

          final requiredUser = snapshot.data!.firstWhere((element) => widget.requestedUserIDs.contains(element.id));
          return Text(
            '${requiredUser.title} ${requiredUser.firstName} ${requiredUser.lastName} bitte melden Sie sich an',
            style: const TextStyle(
              fontSize: 50,
              fontWeight: FontWeight.bold,
            ),
          );
        },
      ),
    );
  }
}
