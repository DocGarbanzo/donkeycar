# Claude Code GitHub Actions Setup

This repository includes GitHub Actions workflows to enable Claude to automatically handle issues and review pull requests.

## Required Setup Steps

### 1. Add Repository Secrets

Go to your repository Settings → Secrets and variables → Actions, and add:

- `ANTHROPIC_API_KEY`: Your Anthropic API key for Claude access
  - Get this from: https://console.anthropic.com/
  - Should start with `sk-ant-`

### 2. Workflow Triggers

#### Issue Handling (`claude-code-issues.yml`)
Triggers when:
- An issue is labeled with `claude`
- A comment contains `@claude`

#### PR Review (`claude-code-pr-review.yml`)  
Triggers when:
- A PR is labeled with `claude-review`
- A PR review comment contains `@claude-review`

### 3. Usage Examples

#### Auto-handle an Issue
1. Create or comment on an issue
2. Add the `claude` label OR mention `@claude` in a comment
3. Claude will analyze and respond with solutions

#### Request PR Review
1. Create a pull request
2. Add the `claude-review` label OR comment `@claude-review`
3. Claude will review code quality, bugs, performance, and standards

### 4. Repository Permissions

The workflows require these permissions (already configured):
- `issues: write` - To comment on issues
- `pull-requests: write` - To comment on PRs
- `contents: read/write` - To access repository code

### 5. Customization

Both workflows reference the project's `CLAUDE.md` file for:
- Coding standards and guidelines
- Project-specific context
- Development patterns

To customize Claude's behavior, update the prompts in the workflow files or modify `CLAUDE.md`.

## Troubleshooting

- Ensure `ANTHROPIC_API_KEY` is correctly set in repository secrets
- Check that workflow permissions are enabled in repository settings
- Verify labels (`claude`, `claude-review`) are created in your repository
- Review GitHub Actions logs for any errors

## Security Notes

- API keys are stored securely in GitHub Secrets
- Workflows only trigger on specific labels/mentions
- All actions are logged and auditable
- Claude responses are posted as comments, not committed directly