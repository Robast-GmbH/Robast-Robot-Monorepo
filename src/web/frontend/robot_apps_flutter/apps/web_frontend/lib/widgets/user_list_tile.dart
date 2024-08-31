import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';

class UserListTile extends StatelessWidget {
  const UserListTile({required this.user, this.onDelete, super.key});
  final User user;
  final VoidCallback? onDelete;
  @override
  Widget build(BuildContext context) {
    return ListTile(
      title: Text('${user.firstName} ${user.lastName}'),
      trailing: IconButton(
          onPressed: () async {
            final wasUserDeleted = await Provider.of<UserProvider>(context, listen: false).deleteUser(userID: user.id);
            if (wasUserDeleted) {
              onDelete?.call();
            }
          },
          icon: Icon(Icons.delete)),
    );
  }
}
