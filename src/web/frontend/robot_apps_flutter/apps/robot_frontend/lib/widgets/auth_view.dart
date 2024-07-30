import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

class AuthView extends StatefulWidget {
  const AuthView({
    required this.requiredUserIDs,
    required this.requiredUserGroups,
    required this.onAuthCompleted,
    this.onRetry,
    super.key,
  });
  final List<String> requiredUserIDs;
  final List<String> requiredUserGroups;
  final void Function(bool) onAuthCompleted;
  final VoidCallback? onRetry;

  @override
  State<AuthView> createState() => _AuthViewState();
}

class _AuthViewState extends State<AuthView> {
  late Future<List<User>> loadUsers;
  late Future<bool> tryStartUserSessionFuture;
  Timer? timeoutTimer;

  Future<bool> tryStartUserSession() async {
    final wasSuccessful = await Provider.of<UserProvider>(context, listen: false).tryStartUserSession(
      robotName: 'rb_theron',
      requiredUserIDs: widget.requiredUserIDs,
      requiredUserGroups: widget.requiredUserGroups,
    );
    widget.onAuthCompleted(wasSuccessful);

    return wasSuccessful;
  }

  @override
  void initState() {
    super.initState();
    if (widget.requiredUserIDs.isNotEmpty || widget.requiredUserGroups.isNotEmpty) {
      loadUsers = Provider.of<UserProvider>(context, listen: false).getUsers();
    }
    tryStartUserSessionFuture = tryStartUserSession();
  }

  @override
  void dispose() {
    timeoutTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        if (widget.requiredUserIDs.isEmpty && (widget.requiredUserGroups.isEmpty || widget.requiredUserGroups.length == 3))
          const Text(
            'Bitte authentifizieren Sie sich mit Ihrem NFC-Tag',
            style: TextStyle(
              fontSize: 50,
              fontWeight: FontWeight.bold,
            ),
          )
        else
          Center(
            child: FutureBuilder<List<User>>(
              future: loadUsers,
              builder: (context, snapshot) {
                if (snapshot.connectionState != ConnectionState.done) {
                  return const CircularProgressIndicator();
                }

                if (widget.requiredUserIDs.isEmpty && widget.requiredUserGroups.isNotEmpty) {
                  return Text(
                    'Bitte melden Sie sich an ${widget.requiredUserGroups.join(', ')}',
                    style: const TextStyle(
                      fontSize: 50,
                      fontWeight: FontWeight.bold,
                    ),
                  );
                }

                final requiredUser = snapshot.data!.firstWhere((element) => widget.requiredUserIDs.contains(element.id));
                return Text(
                  '${requiredUser.title} ${requiredUser.firstName} ${requiredUser.lastName} bitte melden Sie sich an',
                  style: const TextStyle(
                    fontSize: 50,
                    fontWeight: FontWeight.bold,
                  ),
                );
              },
            ),
          ),
        const SizedBox(
          height: 32,
        ),
        FutureBuilder<bool>(
          future: tryStartUserSessionFuture,
          builder: (context, snapshot) {
            if (snapshot.connectionState != ConnectionState.done) {
              return const CircularProgressIndicator();
            } else if (snapshot.data!) {
              return const Text(
                'Authentifizierung erfolgreich',
                style: TextStyle(
                  fontSize: 50,
                  fontWeight: FontWeight.bold,
                ),
              );
            } else {
              return InkWell(
                onTap: () {
                  tryStartUserSessionFuture = tryStartUserSessionFuture = tryStartUserSession();
                  widget.onRetry?.call();
                  setState(() {});
                },
                child: const Text(
                  'Authentifizierung fehlgeschlagen',
                  style: TextStyle(
                    fontSize: 50,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              );
            }
          },
        ),
      ],
    );
  }
}
