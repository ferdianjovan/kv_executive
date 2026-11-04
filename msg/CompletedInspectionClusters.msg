# Authoritative bounded snapshot; an empty list clears all completed clusters.
# Header stamp orders snapshots. Historical center measurement times are retained.
std_msgs/Header header
CompletedInspectionCluster[] clusters
