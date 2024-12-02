import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/hygiene_provider.dart';
import 'package:robot_frontend/widgets/notification_tile.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class NotificationView extends StatelessWidget {
  const NotificationView({super.key});

  @override
  Widget build(BuildContext context) {
    final hygieneProvider = Provider.of<HygieneProvider>(context, listen: true);
    return RoundedContainer(
      child: SizedBox(
        width: double.infinity,
        height: double.infinity,
        child: Padding(
            padding: const EdgeInsets.all(16),
            child: ListView(
              children: [if (hygieneProvider.requiresCleaning ?? false) const NotificationTile()],
            )),
      ),
    );
  }
}
