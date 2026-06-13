import argparse
import os
import subprocess
import glob
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SHARED_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'shared'))
QUEUE_DIR = os.path.join(SHARED_DIR, 'queue')
REVIEWS_DIR = os.path.join(SHARED_DIR, 'reviews')
CONTEXT_DIR = os.path.join(SHARED_DIR, 'context')

FLAG_FILE = os.path.join(QUEUE_DIR, 'REVIEW_REQUESTED.flag')
PENDING_FILE = os.path.join(QUEUE_DIR, 'PENDING_REVIEW.md')
TEMPLATE_FILE = os.path.join(CONTEXT_DIR, 'PENDING_REVIEW_TEMPLATE.md')

def run_cmd(cmd):
    try:
        return subprocess.check_output(cmd, shell=True, text=True, stderr=subprocess.STDOUT).strip()
    except subprocess.CalledProcessError as e:
        return f"[Error: {e.output.strip()}]"

def generate(notes):
    branch = run_cmd("git rev-parse --abbrev-ref HEAD")
    commit = run_cmd("git rev-parse --short HEAD")
    
    changed_files = run_cmd("git diff --name-only main...HEAD")
    if "[Error" in changed_files or not changed_files:
        changed_files = run_cmd("git diff --name-only HEAD~1 HEAD")
        if "[Error" in changed_files:
            changed_files = "No files found or error."

    files_list = "\n".join([f"- `{f}`" for f in changed_files.split('\n') if f])

    if not notes:
        notes = "[TODO: Add concerns]"

    if not os.path.exists(TEMPLATE_FILE):
        print(f"Error: Template not found at {TEMPLATE_FILE}")
        return

    with open(TEMPLATE_FILE, 'r', encoding='utf-8') as f:
        content = f.read()

    timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S UTC")
    
    content = content.replace("[Feature Branch Name]", branch)
    content = content.replace("[Latest Commit Hash]", commit)
    content = content.replace("[List of changed files]", files_list)
    content = content.replace("[Agent's self-identified concerns, edge cases, or areas needing close inspection]", notes)
    content = content.replace("[Date/Time]", timestamp)

    os.makedirs(QUEUE_DIR, exist_ok=True)
    with open(PENDING_FILE, 'w', encoding='utf-8') as f:
        f.write(content)
    
    print(f"Generated review request at {PENDING_FILE}")

def trigger():
    os.makedirs(QUEUE_DIR, exist_ok=True)
    with open(FLAG_FILE, 'w', encoding='utf-8') as f:
        pass
    print(f"Triggered review. Created {FLAG_FILE}")

def read_review():
    if not os.path.exists(REVIEWS_DIR):
        print("No reviews directory found.")
        return
        
    reviews = glob.glob(os.path.join(REVIEWS_DIR, "REVIEW_*.md"))
    if not reviews:
        print("No reviews found.")
        return
        
    latest_review = max(reviews, key=os.path.getctime)
    print(f"Reading latest review: {os.path.basename(latest_review)}\n")
    
    with open(latest_review, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    current_section = None
    blockers, majors, minors = [], [], []
    verdict = []
    in_verdict = False
    
    for line in lines:
        stripped = line.strip()
        if "## Verdict" in line:
            in_verdict = True
            continue
        if in_verdict and line.startswith("## "):
            in_verdict = False
            
        if in_verdict and stripped and not stripped.startswith("```"):
            verdict.append(stripped)

        if "BLOCKERS" in line.upper() and ("#" in line):
            current_section = 'blocker'
            continue
        elif "MAJORS" in line.upper() and ("#" in line):
            current_section = 'major'
            continue
        elif "MINORS" in line.upper() and ("#" in line):
            current_section = 'minor'
            continue
        elif line.startswith("## ") or line.startswith("---"):
            if current_section:
                current_section = None

        if current_section and stripped and not stripped.startswith("<!--"):
            if current_section == 'blocker':
                blockers.append(stripped)
            elif current_section == 'major':
                majors.append(stripped)
            elif current_section == 'minor':
                minors.append(stripped)

    print("=== VERDICT ===")
    for v in verdict:
        if v.startswith("[x]") or v.startswith("[X]"):
            print("->", v)
        else:
            print(v)
    print()

    if blockers:
        print("=== BLOCKERS ===")
        for b in blockers: print(b)
        print()
    if majors:
        print("=== MAJORS ===")
        for m in majors: print(m)
        print()
    if minors:
        print("=== MINORS ===")
        for m in minors: print(m)
        print()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AEGIS Review Tool")
    subparsers = parser.add_subparsers(dest='cmd', required=True)
    
    gen_p = subparsers.add_parser('generate', help="Generate PENDING_REVIEW.md")
    gen_p.add_argument('--notes', type=str, default="", help="Agent's self-identified concerns")
    
    trig_p = subparsers.add_parser('trigger', help="Trigger review by creating FLAG_FILE")
    
    read_p = subparsers.add_parser('read', help="Read the latest review")
    
    args = parser.parse_args()
    if args.cmd == 'generate':
        generate(args.notes)
    elif args.cmd == 'trigger':
        trigger()
    elif args.cmd == 'read':
        read_review()
