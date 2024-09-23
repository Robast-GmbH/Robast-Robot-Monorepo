import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/pages/login_page.dart';
import 'package:web_frontend/pages/user_settings_page.dart';
import 'package:web_frontend/widgets/deletion_confirmation_dialog.dart';

class UserListTile extends StatelessWidget {
  const UserListTile({required this.user, this.onDelete, this.onUpdate, super.key});
  final User user;
  final VoidCallback? onDelete;
  final VoidCallback? onUpdate;
  @override
  Widget build(BuildContext context) {
    return ListTile(
      onTap: () async {
        final didUpdateUser = await Navigator.push(
            context,
            MaterialPageRoute<bool?>(
                builder: (context) => UserSettingsPage(
                      user: user,
                      isAdminView: true,
                    )));
        if (didUpdateUser ?? false) {
          onUpdate?.call();
        }
      },
      title: Text('${user.firstName} ${user.lastName}'),
      subtitle: Text((user.mail?.isEmpty ?? true) ? 'Keine E-Mail angegeben' : user.mail!),
      trailing: IconButton(
          onPressed: () async {
            final userProvider = Provider.of<UserProvider>(context, listen: false);
            final bool isCurrentUser = user.id == userProvider.user!.id;
            final isDeletionConfirmed = await showDialog<bool>(
                context: context,
                builder: (context) {
                  return DeletionConfirmationDialog(requiredTextInput: user.lastName);
                });
            if (!(isDeletionConfirmed ?? false)) {
              return;
            }
            final wasUserDeleted = await userProvider.deleteUser(userID: user.id);
            if (isCurrentUser && wasUserDeleted) {
              userProvider.user = null;
              Navigator.pushAndRemoveUntil(
                context,
                MaterialPageRoute(builder: (context) => LoginPage()),
                (Route<dynamic> route) => false,
              );
            }
            if (wasUserDeleted) {
              onDelete?.call();
            }
          },
          icon: Icon(Icons.delete)),
    );
  }
}
