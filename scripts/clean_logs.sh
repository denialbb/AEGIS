#!/bin/bash
# Clean logs and CSV files from the project
cd "$(dirname "$0")/.."
rm -f logs/*.csv logs/*.log logs/latest/*.csv logs/latest/*.log
find . -name "*.csv" -path "*/logs/*" -delete
find . -name "*.log" -path "*/logs/*" -delete
echo "Logs and CSV files cleaned"