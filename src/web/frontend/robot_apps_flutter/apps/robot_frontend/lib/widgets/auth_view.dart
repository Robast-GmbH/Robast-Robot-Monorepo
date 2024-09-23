import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:shared_data_models/shared_data_models.dart';

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
  final void Function({required bool wasSuccessful}) onAuthCompleted;
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
    widget.onAuthCompleted(wasSuccessful: wasSuccessful);

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
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 256),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          if (widget.requiredUserIDs.isEmpty && (widget.requiredUserGroups.isEmpty || widget.requiredUserGroups.length == 3))
            const Text(
              'Bitte authentifizieren Sie sich mit Ihrem NFC-Tag',
              textAlign: TextAlign.center,
              style: TextStyle(fontSize: 80, color: RobotColors.primaryText),
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
                      textAlign: TextAlign.center,
                      style: const TextStyle(fontSize: 80, color: RobotColors.primaryText),
                    );
                  }

                  final requiredUser = snapshot.data!.firstWhere((element) => widget.requiredUserIDs.contains(element.id));
                  return Text(
                    '${requiredUser.title} ${requiredUser.firstName} ${requiredUser.lastName} bitte melden Sie sich an',
                    textAlign: TextAlign.center,
                    style: const TextStyle(fontSize: 80, color: RobotColors.primaryText),
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
              if (snapshot.connectionState != ConnectionState.done || snapshot.data!) {
                return const SizedBox(width: 48, height: 48, child: CircularProgressIndicator());
              } else {
                return InkWell(
                  onTap: () {
                    tryStartUserSessionFuture = tryStartUserSessionFuture = tryStartUserSession();
                    widget.onRetry?.call();
                    setState(() {});
                  },
                  child: const Text(
                    'Authentifizierung fehlgeschlagen',
                    style: TextStyle(fontSize: 80, color: RobotColors.primaryText),
                  ),
                );
              }
            },
          ),
        ],
      ),
    );
  }
}
